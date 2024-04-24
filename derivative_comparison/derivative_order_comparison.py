import numpy as np
import matplotlib.pyplot as plt

# パラメータ設定
n_points = 100
x = np.linspace(0, 10 * np.pi, n_points)
dx = x[1] - x[0]
y = np.sin(x) + np.random.normal(0, 0.1, n_points)  # sin波にホワイトノイズを追加

# 前進差分微分
forward_diff = np.diff(y) / dx
forward_diff = np.append(forward_diff, forward_diff[-1])  # 最後の点の値を保持

# 2次精度中心差分微分
central_diff_2nd = (y[2:] - y[:-2]) / (2 * dx)
central_diff_2nd = np.pad(central_diff_2nd, (1, 1), 'edge')  # 端の値を保持

# 4次精度中心差分微分
central_diff_4th = (-y[4:] + 8*y[3:-1] - 8*y[1:-3] + y[:-4]) / (12 * dx)
central_diff_4th = np.pad(central_diff_4th, (2, 2), 'edge')  # 端の値を保持

# プロット
plt.figure(figsize=(12, 8))
plt.plot(x, y, label='Original signal', alpha=0.7)
# plt.plot(x, forward_diff, label='Forward difference', linestyle='--')
# plt.plot(x, central_diff_2nd, label='2nd order central difference', linestyle='-.')
# plt.plot(x, central_diff_4th, label='4th order central difference', linestyle=':')
plt.plot(x, forward_diff, label='Forward difference')
plt.plot(x, central_diff_2nd, label='2nd order central difference')
plt.plot(x, central_diff_4th, label='4th order central difference')
plt.title('Comparison of Differentiation Methods')
plt.xlabel('x')
plt.ylabel('dy/dx')
plt.legend()
plt.grid(True)
plt.show()