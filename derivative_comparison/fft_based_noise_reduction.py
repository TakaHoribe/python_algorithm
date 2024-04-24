import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft, ifft

# パラメータ設定
fs = 1000  # サンプリング周波数
t = np.arange(0, 1, 1/fs)  # 時間軸
freq = 5  # sin波の周波数

# sin波の生成
clean_signal = np.sin(2 * np.pi * freq * t)

# ノイズを加えた信号
noise = np.random.normal(0, 0.5, len(t))
noisy_signal = clean_signal + noise

# FFTによる周波数ドメインへの変換
fft_result = fft(noisy_signal)

# 周波数軸の作成
freqs = np.fft.fftfreq(len(fft_result), 1/fs)

# 10Hz以上の成分を除去
filtered_fft = fft_result.copy()
filtered_fft[np.abs(freqs) > 10] = 0

# 逆FFTによる時間ドメインへの戻し
recovered_signal = ifft(filtered_fft).real

# プロット
fig, ax = plt.subplots(2, 1, figsize=(12, 10))

# プロット1：元の信号、ノイズ付き信号、フィルタリング後の信号
ax[0].plot(t, clean_signal, label='Original Signal')
ax[0].plot(t, noisy_signal, label='Noisy Signal', alpha=0.7)
ax[0].plot(t, recovered_signal, label='Recovered Signal')
ax[0].legend()
ax[0].set_title('Time Domain Signals')
ax[0].set_xlabel('Time [s]')
ax[0].set_ylabel('Amplitude')
ax[0].grid(True)

# プロット2：FFTの結果
ax[1].plot(freqs, np.abs(fft_result), label='FFT of Noisy Signal')
ax[1].plot(freqs, np.abs(filtered_fft), label='Filtered FFT', alpha=0.7)
ax[1].legend()
ax[1].set_title('Frequency Domain Representation')
ax[1].set_xlabel('Frequency [Hz]')
ax[1].set_ylabel('Magnitude')
ax[1].set_xlim([0, 50])  # 周波数を50Hzまで表示
ax[1].grid(True)

plt.tight_layout()
plt.show()