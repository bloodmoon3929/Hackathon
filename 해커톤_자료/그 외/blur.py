import cv2
import numpy as np

def generate_perlin_noise(width, height, scale=10):
    def f(t):  # 스무딩 함수
        return 6*t**5 - 15*t**4 + 10*t**3

    linx = np.linspace(0, scale, width, endpoint=False)
    liny = np.linspace(0, scale, height, endpoint=False)
    x, y = np.meshgrid(linx, liny)

    xi = x.astype(int)
    yi = y.astype(int)
    xf = x - xi
    yf = y - yi

    u = f(xf)
    v = f(yf)

    rng = np.random.RandomState(42)
    gradients = rng.randn(scale+1, scale+1, 2)
    gradients /= np.linalg.norm(gradients, axis=2, keepdims=True)

    g00 = gradients[xi, yi]
    g10 = gradients[xi+1, yi]
    g01 = gradients[xi, yi+1]
    g11 = gradients[xi+1, yi+1]

    d00 = np.stack([xf, yf], axis=-1)
    d10 = np.stack([xf-1, yf], axis=-1)
    d01 = np.stack([xf, yf-1], axis=-1)
    d11 = np.stack([xf-1, yf-1], axis=-1)

    n00 = np.sum(g00 * d00, axis=2)
    n10 = np.sum(g10 * d10, axis=2)
    n01 = np.sum(g01 * d01, axis=2)
    n11 = np.sum(g11 * d11, axis=2)

    x1 = n00*(1-u) + n10*u
    x2 = n01*(1-u) + n11*u
    result = x1*(1-v) + x2*v

    result = ((result - result.min()) / (result.max() - result.min()) * 255).astype(np.uint8)
    return result

# 원본 이미지 불러오기
img = cv2.imread("before.jpg")
h, w = img.shape[:2]

# Perlin noise 기반 구름 패턴
cloud = generate_perlin_noise(w, h, scale=6)  # scale 낮추면 덩어리 큰 패턴
cloud = cv2.GaussianBlur(cloud, (151, 151), 0)  # 뭉글하게 블러

# 살짝 밝게 조정 (안개는 보통 흰색 계열)
cloud_bright = cv2.normalize(cloud, None, 180, 255, cv2.NORM_MINMAX)

# 3채널 변환
cloud_colored = cv2.merge([cloud_bright, cloud_bright, cloud_bright])

# 원본과 합성 → 안개는 아주 옅게만
fog = cv2.addWeighted(img, 0.98, cloud_colored, 0.02, 0)

# 전체 톤을 살짝 밝게
fog = cv2.convertScaleAbs(fog, alpha=1.0, beta=20)

cv2.imwrite("after.jpg", fog)
