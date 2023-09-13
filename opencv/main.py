import cv2
from numpy import float32

# 获取源图像
src = cv2.imread('opencv/assets/src.jpg', cv2.IMREAD_GRAYSCALE)

# 大津法二值化
_, dst = cv2.threshold(src, 100, 200, cv2.THRESH_OTSU)

# 开闭运算，消除白色黑色噪点
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
dst = cv2.morphologyEx(dst, cv2.MORPH_OPEN, kernel)
dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)

# 获取轮廓
contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# 拟合多边形，四个定点逆时针，getPerspectiveTransform() 函数需要 float32 类型
approxCurve = float32(cv2.approxPolyDP(contours[0], 10, True))
resultPoints = float32([[0, 0],[0, 680],[360, 680],[360,0]])
# 获取透视变换矩阵
transformMat = cv2.getPerspectiveTransform(approxCurve, resultPoints)
# 透视变换
result = cv2.warpPerspective(src, transformMat, (360, 680))

# 可视化结果
cv2.namedWindow("result", cv2.WINDOW_NORMAL)
cv2.imshow("result", result)
cv2.waitKey(0)
