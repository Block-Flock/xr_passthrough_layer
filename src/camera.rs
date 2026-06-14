use std::{
    fs::File,
    path::Path,
    sync::{Arc, mpsc},
    thread::JoinHandle,
    time::Duration,
};

use super::{FrameFormat, FrameInfo};
use anyhow::{Context, anyhow};
use arc_swap::{ArcSwap, Guard};
use glam::UVec2;
use log::{info as debug, warn};
use memmap2::Mmap;
use smallvec::SmallVec;
use v4l::video::Capture;

const PSVR2_HEADER_SIZE: usize = 64;
const PSVR2_MAGIC: &[u8; 8] = b"PSVR2CAM";
const PSVR2_FORMAT_BC4: u32 = 2;
const PSVR2_CALIBRATION_VALID_OFFSET: usize = 48;
const PSVR2_CALIBRATION_VERSION_OFFSET: usize = 52;
const PSVR2_INTRINSICS_OFFSET: usize = 56;
const PSVR2_DISTORTION_OFFSET: usize = 88;
const PSVR2_HEAD_TO_CAMERA_OFFSET: usize = 408;
const PSVR2_CAMERA_POSITION_OFFSET: usize = 536;
const PSVR2_IPD_OFFSET: usize = 568;

#[derive(Clone, Debug)]
pub struct Psvr2Calibration {
    /// fx, fy, cx, cy for each eye.
    pub intrinsics: [[f32; 4]; 2],
    pub distortion: [[f64; 20]; 2],
    /// Column-major 4x4 matrices mapping headset-forward rays into each camera.
    pub head_to_camera: [[[f32; 4]; 4]; 2],
    /// Camera centres in the headset-forward basis, in metres.
    pub camera_position: [[f32; 4]; 2],
    pub ipd_m: f32,
}

impl Psvr2Calibration {
    pub fn rectified_intrinsics(&self) -> [f32; 4] {
        let fx = (self.intrinsics[0][0] + self.intrinsics[1][0]) * 0.5;
        let fy = (self.intrinsics[0][1] + self.intrinsics[1][1]) * 0.5;
        [fx, fy, 508.0, 508.0]
    }
}

#[derive(PartialEq, Eq, Debug, Clone, Copy)]
enum Control {
    /// Pause the camera thread so it's not capturing new frames, but keep the camera device open
    Pause,
    /// Resume capturing
    Resume,
    Exit,
}

pub struct CameraThread {
    frame: Arc<ArcSwap<FrameInfo>>,
    control: mpsc::Sender<Control>,
    join: JoinHandle<()>,
}

impl CameraThread {
    pub fn new(camera: v4l::Device, splash_png: &[u8]) -> Self {
        let (tx, rx) = mpsc::channel();
        let img = image::load_from_memory_with_format(splash_png, image::ImageFormat::Png)
            .unwrap()
            .into_rgba8();
        let extent = [img.width(), img.height()];
        assert!(extent[0].is_multiple_of(2));

        let frame = FrameInfo {
            frame: img.into_raw(),
            frame_time: std::time::Instant::now(),
            capture_time: None,
            capture_timestamp_ns: None,
            ipd_m: None,
            needs_postprocess: false, // splash image doesn't need postprocessing
            size: UVec2::new(extent[0] / 2, extent[1]),
            format: FrameFormat::Rgba8,
        };
        let frame = Arc::new(ArcSwap::new(Arc::new(frame)));
        let join = std::thread::spawn({
            let frame = frame.clone();
            move || Self::run(rx, camera, frame)
        });
        Self {
            frame,
            join,
            control: tx,
        }
    }

    pub fn new_psvr2(path: &Path, splash_png: &[u8]) -> anyhow::Result<(Self, Psvr2Calibration)> {
        let (tx, rx) = mpsc::channel();
        let img = image::load_from_memory_with_format(splash_png, image::ImageFormat::Png)
            .context("cannot decode splash image")?
            .into_rgba8();
        let extent = [img.width(), img.height()];
        if !extent[0].is_multiple_of(2) {
            return Err(anyhow!("Splash image width is not even"));
        }

        let frame = FrameInfo {
            frame: img.into_raw(),
            frame_time: std::time::Instant::now(),
            capture_time: None,
            capture_timestamp_ns: None,
            ipd_m: None,
            needs_postprocess: false,
            size: UVec2::new(extent[0] / 2, extent[1]),
            format: FrameFormat::Rgba8,
        };
        let frame = Arc::new(ArcSwap::new(Arc::new(frame)));
        let stream = Self::wait_for_psvr2(path)?;
        let calibration = stream.calibration.clone();
        let join = std::thread::spawn({
            let frame = frame.clone();
            move || Self::run_psvr2(rx, stream, frame)
        });
        Ok((
            Self {
                frame,
                join,
                control: tx,
            },
            calibration,
        ))
    }

    fn wait_for_psvr2(path: &Path) -> anyhow::Result<Psvr2Stream> {
        const RETRY_MS: u64 = 500;
        const MAX_RETRIES: u32 = 120; // 60 seconds
        for i in 0..MAX_RETRIES {
            match File::open(path) {
                Ok(file) => {
                    let mmap = unsafe { Mmap::map(&file) }.with_context(|| {
                        format!("cannot map PSVR2 camera stream {}", path.display())
                    })?;
                    let stream = Psvr2Stream::new(mmap)?;
                    debug!(
                        "PSVR2 camera stream opened after ~{}ms",
                        i as u64 * RETRY_MS
                    );
                    return Ok(stream);
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound && i < MAX_RETRIES - 1 => {
                    std::thread::sleep(Duration::from_millis(RETRY_MS));
                }
                Err(e) => {
                    return Err(e).with_context(|| {
                        format!("cannot open PSVR2 camera stream {}", path.display())
                    });
                }
            }
        }
        Err(anyhow!(
            "PSVR2 camera stream not available after {} seconds",
            MAX_RETRIES * RETRY_MS as u32 / 1000
        ))
    }

    pub fn frame(&self) -> Guard<Arc<FrameInfo>> {
        self.frame.load()
    }
    pub fn pause(&self) -> anyhow::Result<()> {
        Ok(self.control.send(Control::Pause)?)
    }
    pub fn resume(&self) -> anyhow::Result<()> {
        Ok(self.control.send(Control::Resume)?)
    }
    pub fn exit(self) -> anyhow::Result<()> {
        self.control.send(Control::Exit)?;
        self.join.join().map_err(|e| anyhow!("{e:?}"))
    }

    fn run(control: mpsc::Receiver<Control>, camera: v4l::Device, frame: Arc<ArcSwap<FrameInfo>>) {
        // hold the thread until we get a go signal
        match control.recv() {
            Err(std::sync::mpsc::RecvError) | Ok(Control::Exit) => {
                debug!("camera thread stopped early");
                return;
            }
            Ok(Control::Resume) => (),
            Ok(Control::Pause) => panic!("Invalid pause command"),
        }
        let Err(e) = Self::run_inner(control, camera, frame) else {
            debug!("camera thread stopped");
            return;
        };
        warn!("Camera thread stopped: {e:#}");
    }

    fn run_psvr2(
        control: mpsc::Receiver<Control>,
        mut stream: Psvr2Stream,
        frame: Arc<ArcSwap<FrameInfo>>,
    ) {
        match control.recv() {
            Err(std::sync::mpsc::RecvError) | Ok(Control::Exit) => {
                debug!("PSVR2 camera thread stopped early");
                return;
            }
            Ok(Control::Resume) => (),
            Ok(Control::Pause) => panic!("Invalid pause command"),
        }

        let Err(e) = Self::run_psvr2_inner(control, &mut stream, frame) else {
            debug!("PSVR2 camera thread stopped");
            return;
        };
        warn!("PSVR2 camera thread stopped: {e:#}");
    }

    fn run_psvr2_inner(
        control: mpsc::Receiver<Control>,
        stream: &mut Psvr2Stream,
        frame: Arc<ArcSwap<FrameInfo>>,
    ) -> anyhow::Result<()> {
        let mut last_sequence = 0;
        let mut bc4 = Vec::new();
        loop {
            if let Some(c) = match control.try_recv() {
                Ok(c) => Some(c),
                Err(mpsc::TryRecvError::Empty) => None,
                Err(mpsc::TryRecvError::Disconnected) => break Ok(()),
            } {
                match c {
                    Control::Pause => {
                        let Ok(c) = control.recv() else {
                            break Ok(());
                        };
                        assert_eq!(c, Control::Resume, "unexpected command {c:?}");
                    }
                    Control::Exit => break Ok(()),
                    Control::Resume => panic!("unexpected resume"),
                }
            }

            let Some((capture_time, capture_timestamp_ns, ipd_m)) =
                stream.next_bc4(&mut last_sequence, &mut bc4)?
            else {
                std::thread::sleep(Duration::from_micros(500));
                continue;
            };
            let old_frame = frame.swap(Arc::new(FrameInfo {
                frame: bc4,
                frame_time: capture_time,
                capture_time: Some(capture_time),
                capture_timestamp_ns: Some(capture_timestamp_ns),
                ipd_m: Some(ipd_m),
                needs_postprocess: true,
                size: UVec2::new(stream.width / 2, stream.height),
                format: FrameFormat::Bc4,
            }));
            bc4 = match Arc::try_unwrap(old_frame) {
                Ok(mut old_frame) => {
                    old_frame.frame.clear();
                    old_frame.frame
                }
                Err(_) => Vec::new(),
            };
        }
    }

    fn run_inner(
        control: mpsc::Receiver<Control>,
        camera: v4l::Device,
        frame: Arc<ArcSwap<FrameInfo>>,
    ) -> anyhow::Result<()> {
        let mut first_frame_time = None;
        // We want to make the latency as low as possible, so only set a single buffer.
        let mut video_stream =
            v4l::prelude::MmapStream::with_buffers(&camera, v4l::buffer::Type::VideoCapture, 1)
                .context("cannot open camera mmap stream")?;
        let mut is_splash = true;
        const MAX_POOL_SIZE: usize = 2;
        let mut pool = SmallVec::<[Arc<FrameInfo>; 2]>::new();
        let camera_format = camera.format()?;
        if camera_format.width % 2 != 0 {
            return Err(anyhow!("Camera width is not even"));
        }

        let find_free = |pool: &mut SmallVec<_>| {
            // Find unused image from pool
            let index = pool.iter_mut().position(|i| Arc::get_mut(i).is_some())?;
            Some(pool.swap_remove(index))
        };
        loop {
            if let Some(c) = match control.try_recv() {
                Ok(c) => Some(c),
                Err(mpsc::TryRecvError::Empty) => None,
                Err(mpsc::TryRecvError::Disconnected) => break Ok(()),
            } {
                match c {
                    Control::Pause => {
                        let Ok(c) = control.recv() else {
                            break Ok(());
                        };
                        assert_eq!(c, Control::Resume, "unexpected command {c:?}");
                    }
                    Control::Exit => break Ok(()),
                    Control::Resume => panic!("unexpected resume"),
                }
            }
            log::trace!("getting camera frame");
            let (frame_data, metadata) = v4l::io::traits::CaptureStream::next(&mut video_stream)?;
            if metadata.flags.contains(v4l::buffer::Flags::ERROR) {
                continue;
            }
            let frame_time = if let Some((camera_reference, reference)) = first_frame_time {
                let camera_elapsed =
                    std::time::Duration::from(metadata.timestamp) - camera_reference;
                reference + camera_elapsed
            } else {
                let now = std::time::Instant::now();
                first_frame_time = Some((metadata.timestamp.into(), now));
                now
            };
            log::trace!("got camera frame {:?}", frame_time);
            let new_frame = find_free(&mut pool)
                .map(|mut fi| {
                    log::trace!("Reusing frame");
                    let mfi = Arc::get_mut(&mut fi).unwrap();
                    mfi.frame.copy_from_slice(frame_data);
                    mfi.frame_time = frame_time;
                    mfi.capture_time = None;
                    mfi.capture_timestamp_ns = None;
                    mfi.ipd_m = None;
                    mfi.needs_postprocess = true;
                    fi
                })
                .unwrap_or_else(|| {
                    log::debug!("Allocated new frame");
                    FrameInfo {
                        frame: frame_data.to_vec(),
                        frame_time: std::time::Instant::now(),
                        capture_time: None,
                        capture_timestamp_ns: None,
                        ipd_m: None,
                        needs_postprocess: true,
                        size: UVec2::new(camera_format.width / 2, camera_format.height),
                        format: FrameFormat::Yuyv,
                    }
                    .into()
                });
            let old_frame = frame.swap(new_frame);
            // splash image isn't necessarily `frame_data` sized, so don't reuse it.
            // if we already have too many frames, just release the old one.
            if !is_splash && pool.len() < MAX_POOL_SIZE {
                pool.push(old_frame);
            } else {
                log::info!("Releasing frame image, is_splash {is_splash}");
            }
            is_splash = false;
            // log::debug!("got camera frame {}", frame_data.len());
        }
    }
}

struct Psvr2Stream {
    mmap: Mmap,
    width: u32,
    height: u32,
    stride: u32,
    header_size: usize,
    calibration: Psvr2Calibration,
}

impl Psvr2Stream {
    fn read_u32(mmap: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(mmap[offset..offset + 4].try_into().unwrap())
    }

    fn read_f32(mmap: &[u8], offset: usize) -> f32 {
        f32::from_le_bytes(mmap[offset..offset + 4].try_into().unwrap())
    }

    fn read_f64(mmap: &[u8], offset: usize) -> f64 {
        f64::from_le_bytes(mmap[offset..offset + 8].try_into().unwrap())
    }

    fn new(mmap: Mmap) -> anyhow::Result<Self> {
        if mmap.len() < PSVR2_HEADER_SIZE || &mmap[..8] != PSVR2_MAGIC {
            return Err(anyhow!("invalid PSVR2 camera stream header"));
        }
        let version = Self::read_u32(&mmap, 8);
        let width = Self::read_u32(&mmap, 12);
        let height = Self::read_u32(&mmap, 16);
        let stride = Self::read_u32(&mmap, 20);
        let format = Self::read_u32(&mmap, 24);
        let header_size = Self::read_u32(&mmap, 28) as usize;
        if version != 2 || format != PSVR2_FORMAT_BC4 {
            return Err(anyhow!(
                "unsupported PSVR2 camera stream version {version}, format {format}"
            ));
        }
        if width != 2032 || height != 1016 || stride < width || header_size < PSVR2_HEADER_SIZE {
            return Err(anyhow!(
                "unexpected PSVR2 stream geometry {width}x{height}, stride {stride}, header {header_size}"
            ));
        }
        let required_len = header_size
            .checked_add(stride as usize * (height as usize / 4))
            .context("PSVR2 stream size overflow")?;
        if mmap.len() < required_len {
            return Err(anyhow!(
                "PSVR2 camera stream is truncated: {} bytes, need {required_len}",
                mmap.len()
            ));
        }
        if header_size <= PSVR2_IPD_OFFSET + 4
            || Self::read_u32(&mmap, PSVR2_CALIBRATION_VALID_OFFSET) != 1
            || Self::read_u32(&mmap, PSVR2_CALIBRATION_VERSION_OFFSET) != 1
        {
            return Err(anyhow!(
                "PSVR2 camera stream has no usable factory calibration"
            ));
        }

        let mut calibration = Psvr2Calibration {
            intrinsics: [[0.0; 4]; 2],
            distortion: [[0.0; 20]; 2],
            head_to_camera: [[[0.0; 4]; 4]; 2],
            camera_position: [[0.0; 4]; 2],
            ipd_m: Self::read_f32(&mmap, PSVR2_IPD_OFFSET),
        };
        for eye in 0..2 {
            for i in 0..4 {
                calibration.intrinsics[eye][i] =
                    Self::read_f32(&mmap, PSVR2_INTRINSICS_OFFSET + (eye * 4 + i) * 4);
            }
            for i in 0..20 {
                calibration.distortion[eye][i] =
                    Self::read_f64(&mmap, PSVR2_DISTORTION_OFFSET + (eye * 20 + i) * 8);
            }
            for column in 0..4 {
                for row in 0..4 {
                    calibration.head_to_camera[eye][column][row] = Self::read_f32(
                        &mmap,
                        PSVR2_HEAD_TO_CAMERA_OFFSET + (eye * 16 + column * 4 + row) * 4,
                    );
                }
            }
            for i in 0..4 {
                calibration.camera_position[eye][i] =
                    Self::read_f32(&mmap, PSVR2_CAMERA_POSITION_OFFSET + (eye * 4 + i) * 4);
            }
        }
        debug!(
            "Using calibrated PSVR2 stereo camera stream: {}x{}, stride {}, IPD {:.1}mm",
            width,
            height,
            stride,
            calibration.ipd_m * 1000.0
        );
        Ok(Self {
            mmap,
            width,
            height,
            stride,
            header_size,
            calibration,
        })
    }

    fn sequence(&self) -> u64 {
        // The producer uses release stores around each frame. Volatile reads prevent the
        // compiler from caching the sequence while we copy from the shared mapping.
        unsafe { std::ptr::read_volatile(self.mmap.as_ptr().add(32).cast::<u64>()) }
    }

    fn timestamp_ns(&self) -> u64 {
        unsafe { std::ptr::read_volatile(self.mmap.as_ptr().add(40).cast::<u64>()) }
    }

    fn ipd_m(&self) -> f32 {
        unsafe { std::ptr::read_volatile(self.mmap.as_ptr().add(PSVR2_IPD_OFFSET).cast::<f32>()) }
    }

    fn capture_instant(timestamp_ns: u64) -> std::time::Instant {
        let mut now = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        let result = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
        if result != 0 {
            return std::time::Instant::now();
        }

        let now_ns = (now.tv_sec as u64)
            .saturating_mul(1_000_000_000)
            .saturating_add(now.tv_nsec as u64);
        let age = Duration::from_nanos(now_ns.saturating_sub(timestamp_ns));
        std::time::Instant::now()
            .checked_sub(age)
            .unwrap_or_else(std::time::Instant::now)
    }

    fn next_bc4(
        &mut self,
        last_sequence: &mut u64,
        bc4: &mut Vec<u8>,
    ) -> anyhow::Result<Option<(std::time::Instant, u64, f32)>> {
        let sequence = self.sequence();
        if sequence == 0 || sequence & 1 != 0 || sequence == *last_sequence {
            return Ok(None);
        }
        std::sync::atomic::fence(std::sync::atomic::Ordering::Acquire);
        let timestamp_ns = self.timestamp_ns();
        let ipd_m = self.ipd_m();
        let data_len = self.stride as usize * (self.height as usize / 4);
        bc4.resize(data_len, 0);
        bc4.copy_from_slice(&self.mmap[self.header_size..self.header_size + data_len]);
        std::sync::atomic::fence(std::sync::atomic::Ordering::Acquire);
        if self.sequence() != sequence {
            return Ok(None);
        }
        *last_sequence = sequence;

        Ok(Some((
            Self::capture_instant(timestamp_ns),
            timestamp_ns,
            ipd_m,
        )))
    }
}
