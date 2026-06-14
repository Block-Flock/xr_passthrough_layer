#version 450
// Switches:
//
// INPUT_IS_YUYV: when set the input texture is in yuyv pixel format, otherwise
// it is RGB (or is yuyv but converted by the VK_KHR_sampler_ycbcr_conversion extension),
// and `sample_yuyv2` will be no-op.
// UNDISTORT: whether to apply lens undistortion.

layout(binding = 0) uniform Transform {
    mat4 mvp[2];
    float overlayWidth;
    vec2 eyeOffset;
};
layout(binding = 1) uniform sampler2D inputTex;
#ifdef UNDISTORT
layout(binding = 2) uniform DistortionParameters {
    // Parameters are per-eye
    // 0 = left, 1 = right

    // Distortion coefficients
    vec4 dcoef[2];
    // Optical center
    vec2 center[2];
    // Focal length in terms of focal divided by sensor_width
    // 0 = left, 1 = right
    vec2 focal[2];
    // Scaling of the output image
    // 0 = left, 1 = right
    vec2 scale[2];
    // Pixel size of the sensor_width
    //float sensorSize;
};
#endif
#ifdef PSVR2_RECTIFY
layout(binding = 2) uniform Psvr2Calibration {
    // fx, fy, cx, cy in active 1016x1016 camera pixels.
    vec4 psvr2Intrinsics[2];
    // Five vec4s per eye, preserving the driver's 20-coefficient ordering.
    vec4 psvr2Coefficients[10];
    // Factory headset-forward to camera transforms, stored column-major.
    mat4 psvr2HeadToCamera[2];
    // Camera centres in the headset-forward basis, in metres.
    vec4 psvr2CameraPosition[2];
    // x = current IPD in metres, y = optional constant convergence depth.
    vec4 psvr2EyeAndDepth;
    // Shared virtual camera used by both rectified output eyes.
    vec4 psvr2RectifiedIntrinsics;
    // brightness, contrast, unsharp amount, unsharp radius in camera pixels.
    vec4 psvr2ImageQuality;
};
#endif

// 0 = left, 1 = right
layout(location = 0) in flat uint instanceId;
// Input coordinates -1 ~ 1
// relative to the center of the undistorted image for the given eye,
// this way we align the optical center to the center of the image.
layout(location = 1) in vec2 eyeRelativeCoord;

layout(location = 0) out vec4 color;

// stages: yuyv -> rgb -> distortion corrected -> projected

#ifdef INPUT_IS_YUYV
// bt709 -> rgb conversion matrix
const mat3 yuvMatrix = mat3(
    1.164,  1.164, 1.164,
    0.000, -0.392, 2.017,
    1.596, -0.813, 0.000
);

vec4 sample_pixel(vec2 coord) { // `coord` in `rgb` coord system, and not normalized
    coord = coord + vec2(0, 0.5); // align x to the left-center of the pixel

    vec2 size = vec2(textureSize(inputTex, 0));
    vec2 tex_coord = vec2(floor(coord.x / 2.0) + 0.5, coord.y) / size;
    vec4 yuyv = texture(inputTex, tex_coord);
    vec3 yuv;
    if (mod(coord.x, 2.0) == 0) {
        yuv = vec3(yuyv.xyw);
    } else {
        yuv = vec3(yuyv.zyw);
    }
    yuv -= vec3(0.0625, 0.5, 0.5);
    return vec4(yuvMatrix * yuv, 1.0);
}

vec4 sample_input(vec2 coord) { // `coord` in `rgb` coord system
    // coord is normalized, but we actually need pixel coordinates
    coord *= textureSize(inputTex, 0) * vec2(2.0, 1.0);
    vec2 f = fract(coord);
    coord = floor(coord);
    vec4 color = vec4(0.0);

    // Linear intepolation
    const vec3 offsets[4] = {
        vec3(0.0, 0.0, (1 - f.x) * (1 - f.y)),
        vec3(0.0, 1.0, (1 - f.x) * f.y),
        vec3(1.0, 0.0, f.x * (1 - f.y)),
        vec3(1.0, 1.0, f.x * f.y),
    };
    for (int i = 0; i < 4; i++) {
        color += sample_pixel(coord + offsets[i].xy) * offsets[i].z;
    }

    return color;
}
#elif defined(PSVR2_RECTIFY)
// BC4 is single-channel compressed. Sampling gives (r, 0, 0) — replicate r for grayscale.
vec4 sample_input(vec2 coord) {
    float v = texture(inputTex, coord).r;
    return vec4(v, v, v, 1.0);
}
#else
vec4 sample_input(vec2 coord) {
    vec4 rgb = texture(inputTex, coord);
    return vec4(rgb.rgb, 1.0);
}
#endif

#ifdef PSVR2_RECTIFY
float psvr2_coefficient(uint eye, uint index) {
    vec4 packed = psvr2Coefficients[eye * 5u + index / 4u];
    return packed[index % 4u];
}

vec2 psvr2_distort(vec2 point, uint eye) {
    float x = point.x;
    float y = point.y;
    float xSq = x * x;
    float ySq = y * y;
    float rSq = xSq + ySq;
    float twoXY = 2.0 * x * y;
    float rSqP4 = rSq * rSq * rSq * rSq;
    float rSqP5 = rSqP4 * rSq;

    float numPoly = 1.0
        + (((psvr2_coefficient(eye, 4u) * rSq + psvr2_coefficient(eye, 1u)) * rSq
            + psvr2_coefficient(eye, 0u)) * rSq)
        + psvr2_coefficient(eye, 16u) * rSqP5 * rSq
        + psvr2_coefficient(eye, 15u) * rSqP5
        + psvr2_coefficient(eye, 14u) * rSqP4;
    float denPoly = 1.0
        + (((psvr2_coefficient(eye, 7u) * rSq + psvr2_coefficient(eye, 6u)) * rSq
            + psvr2_coefficient(eye, 5u)) * rSq)
        + psvr2_coefficient(eye, 19u) * rSqP5 * rSq
        + psvr2_coefficient(eye, 18u) * rSqP5
        + psvr2_coefficient(eye, 17u) * rSqP4;
    float radialScale = abs(denPoly) > 1e-9 ? numPoly / denPoly : 1.0;

    vec3 distorted = vec3(
        ((psvr2_coefficient(eye, 9u) * rSq + psvr2_coefficient(eye, 8u)) * rSq)
            + (2.0 * xSq + rSq) * psvr2_coefficient(eye, 3u)
            + radialScale * x
            + twoXY * psvr2_coefficient(eye, 2u),
        ((psvr2_coefficient(eye, 11u) * rSq + psvr2_coefficient(eye, 10u)) * rSq)
            + twoXY * psvr2_coefficient(eye, 3u)
            + (2.0 * ySq + rSq) * psvr2_coefficient(eye, 2u)
            + radialScale * y,
        1.0
    );

    float ax = psvr2_coefficient(eye, 12u);
    float ay = psvr2_coefficient(eye, 13u);
    float cx = cos(ax);
    float sx = sin(ax);
    float cy = cos(ay);
    float sy = sin(ay);
    vec3 rotatedX = vec3(
        distorted.x,
        cx * distorted.y - sx * distorted.z,
        sx * distorted.y + cx * distorted.z
    );
    vec3 rotated = vec3(
        cy * rotatedX.x + sy * rotatedX.z,
        rotatedX.y,
        -sy * rotatedX.x + cy * rotatedX.z
    );
    return rotated.xy / rotated.z;
}

float psvr2_sample_gray(vec2 sourcePixel, uint eye) {
    vec2 pixel = clamp(sourcePixel, vec2(0.0), vec2(1015.0));
    vec2 sourceUv = vec2(
        (pixel.x + float(eye) * 1016.0 + 0.5) / 2032.0,
        (pixel.y + 0.5) / 1016.0
    );
    return texture(inputTex, sourceUv).r;
}

vec4 psvr2_project(vec2 coord, uint eye) {
    vec2 outputUv = coord * 0.5 + 0.5;
    vec4 intr = psvr2Intrinsics[eye];
    vec3 headRay = vec3(
        (outputUv.x * 1016.0 - psvr2RectifiedIntrinsics.z) / psvr2RectifiedIntrinsics.x,
        (outputUv.y * 1016.0 - psvr2RectifiedIntrinsics.w) / psvr2RectifiedIntrinsics.y,
        1.0
    );

    vec3 cameraRay;
    float convergenceDepth = psvr2EyeAndDepth.y;
    if (convergenceDepth > 0.0) {
        float eyeX = (eye == 0u ? -0.5 : 0.5) * psvr2EyeAndDepth.x;
        vec3 worldPoint = vec3(eyeX, 0.0, 0.0) + headRay * convergenceDepth;
        cameraRay = mat3(psvr2HeadToCamera[eye])
            * (worldPoint - psvr2CameraPosition[eye].xyz);
    } else {
        cameraRay = mat3(psvr2HeadToCamera[eye]) * headRay;
    }
    if (cameraRay.z <= 1e-5) {
        return vec4(0.0);
    }

    vec2 distorted = psvr2_distort(cameraRay.xy / cameraRay.z, eye);
    vec2 sourcePixel = distorted * intr.xy + intr.zw;
    if (any(lessThan(sourcePixel, vec2(0.0)))
        || sourcePixel.x >= 1016.0 || sourcePixel.y >= 1016.0) {
        return vec4(0.0);
    }

    float raw = psvr2_sample_gray(sourcePixel, eye);
    float radius = psvr2ImageQuality.w;
    float blurred =
        psvr2_sample_gray(sourcePixel + vec2(-radius, -radius), eye) * (1.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(0.0, -radius), eye) * (2.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(radius, -radius), eye) * (1.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(-radius, 0.0), eye) * (2.0 / 16.0)
        + raw * (4.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(radius, 0.0), eye) * (2.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(-radius, radius), eye) * (1.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(0.0, radius), eye) * (2.0 / 16.0)
        + psvr2_sample_gray(sourcePixel + vec2(radius, radius), eye) * (1.0 / 16.0);
    float gray = clamp(raw + psvr2ImageQuality.z * (raw - blurred), 0.0, 1.0);
    gray = clamp(gray * psvr2ImageQuality.x, 0.0, 1.0);
    gray = clamp(0.5 + (gray - 0.5) * psvr2ImageQuality.y, 0.0, 1.0);
    return vec4(gray, gray, gray, 1.0);
}
#endif

#ifdef UNDISTORT
vec4 undistort(vec2 coord, uint eyeIndex) {
    float texOffsetX = 0.5 * float(eyeIndex);

    // coord.y *= -1 ??
    vec2 r = coord * scale[eyeIndex] / focal[eyeIndex] / 2.0;
    // Also scale the r so the whole circular region will be included
    // in the output.
    float theta = atan(length(r));
    float theta2 = theta * theta;
    vec4 c = dcoef[eyeIndex];

    theta *= 1 + theta2 * (c.x +
                 theta2 * (c.y +
                 theta2 * (c.z +
                 theta2 * c.w)));
    // Scale r vector to length theta
    vec2 mapped = theta / length(r) * r;
    mapped *= focal[eyeIndex];
    // mapped should now be -0.5~0.5, in inputTex coord
    // move mapped so its centered at `center`
    mapped = mapped + center[eyeIndex];
    // mapped is now 0 ~ 1
    // scale x by 0.5 because inputTex is 2 image side by side
    mapped.x *= 0.5;
    mapped.x += texOffsetX;
    // mapped is now (0~0.5, 0~1.0);

    return sample_input(mapped);
}
#else
vec4 undistort(vec2 coord, float texOffsetX) {
    vec2 eyeCoord = coord * 0.5 + 0.5;
#ifdef PSVR2_RECTIFY
    return psvr2_project(coord, instanceId);
#endif
    eyeCoord = clamp(eyeCoord, vec2(0.0), vec2(1.0));
    return sample_input(vec2((eyeCoord.x + float(instanceId)) * 0.5, eyeCoord.y));
}
#endif

void main() {
    color = undistort(eyeRelativeCoord, instanceId);
}
