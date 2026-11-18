"""
Sensirion Gas Index Algorithm (v3.2.0) Port for Python / MicroPython

This file contains a Python/MicroPython port of the official Sensirion
Gas Index Algorithm. It processes raw SGP40/SGP41 sensor data to calculate
VOC and NOx indices.
"""

# ==============================================================================
# LICENSE (MIT)
# ==============================================================================
# Copyright (c) 2024 Roman Shevchik <kolbasilyvasily@yandex.ru>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# SPDX-License-Identifier: MIT

# ==============================================================================
# ATTRIBUTION: Original Algorithm by Sensirion AG (BSD-3-Clause)
# ==============================================================================
# This port is based on the official Sensirion Gas Index Algorithm v3.2.0
# (https://github.com/Sensirion/gas-index-algorithm)
#
# Original Copyright (c) 2022, Sensirion AG
# Original License: 3-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ==============================================================================

import math
try:
    from micropython import const
except ImportError:
    def const(val):
        return val

# Постоянные алгоритма Sensirion v3.2.0
ALGORITHM_TYPE_VOC = 0
ALGORITHM_TYPE_NOX = 1

DEFAULT_SAMPLING_INTERVAL = 1.0
INITIAL_BLACKOUT = 45.0
_INDEX_GAIN = 230.0
_SRAW_STD_INITIAL = 50.0
_SRAW_STD_BONUS_VOC = 220.0
_SRAW_STD_NOX = 2000.0

_TAU_MEAN_HOURS = 12.0
_TAU_VARIANCE_HOURS = 12.0
_TAU_INITIAL_MEAN_VOC = 20.0
_TAU_INITIAL_MEAN_NOX = 1200.0
_TAU_INITIAL_VARIANCE = 2500.0

_SEC_IN_MIN = const(60)
_SEC_IN_HOUR = const(3600)
_INIT_DURATION_MEAN_VOC = _SEC_IN_HOUR * 0.75
_INIT_DURATION_MEAN_NOX = _SEC_IN_HOUR * 4.75
_INIT_DURATION_VARIANCE_VOC = _SEC_IN_HOUR * 1.45
_INIT_DURATION_VARIANCE_NOX = _SEC_IN_HOUR * 5.70

_INIT_TRANSITION_MEAN = 0.01
_INIT_TRANSITION_VARIANCE = 0.01

_GATING_THRESHOLD_VOC = 340.0
_GATING_THRESHOLD_NOX = 30.0
_GATING_THRESHOLD_INITIAL = 510.0
_GATING_THRESHOLD_TRANSITION = 0.09
_GATING_MAX_RATIO = 0.3
_GATING_VOC_MAX_DURATION_MINUTES = _SEC_IN_MIN * 3.0   # 180.0
_GATING_NOX_MAX_DURATION_MINUTES = _SEC_IN_MIN * 12.0  # 720.0

_SIGMOID_L = 500.0
_SIGMOID_K_VOC = -0.0065
_SIGMOID_X0_VOC = 213.0
_SIGMOID_K_NOX = -0.0101
_SIGMOID_X0_NOX = 614.0

_VOC_INDEX_OFFSET_DEFAULT = 100.0
_NOX_INDEX_OFFSET_DEFAULT = 1.0

_LP_TAU_FAST = 20.0
_LP_TAU_SLOW = 500.0
_LP_ALPHA = -0.2

_VOC_SRAW_MINIMUM = 20000
_NOX_SRAW_MINIMUM = 10000

_MVE_GAMMA_SCALING = 64.0
_MVE_ADDITIONAL_GAMMA_MEAN_SCALING = 8.0
_MVE_FIX16_MAX = 32767.0
# дополнительно
_X1 = const(32767)
_X2 = 1440.0


class GasIndexAlgorithm:
    """
    Порт официального алгоритма Sensirion Gas Index Algorithm (VOC / NOx).
    Гарантирует бит-в-бит совпадение результатов с эталонной C-библиотекой v3.2.0.
    """

    def __init__(self, algorithm_type: int = ALGORITHM_TYPE_VOC, sampling_interval: float = 1.0):
        if algorithm_type not in (ALGORITHM_TYPE_VOC, ALGORITHM_TYPE_NOX):
            raise ValueError(f"Invalid algorithm type: {algorithm_type}. Use 0 (VOC) or 1 (NOx).")

        self.algorithm_type = algorithm_type
        self.sampling_interval = float(sampling_interval)

        # Параметры, зависящие от типа газа
        if self.algorithm_type == ALGORITHM_TYPE_NOX:
            self.index_offset = _NOX_INDEX_OFFSET_DEFAULT
            self.sraw_minimum = _NOX_SRAW_MINIMUM
            self.gating_max_duration_minutes = _GATING_NOX_MAX_DURATION_MINUTES
            self.init_duration_mean = _INIT_DURATION_MEAN_NOX
            self.init_duration_variance = _INIT_DURATION_VARIANCE_NOX
            self.gating_threshold = _GATING_THRESHOLD_NOX
        else:
            self.index_offset = _VOC_INDEX_OFFSET_DEFAULT
            self.sraw_minimum = _VOC_SRAW_MINIMUM
            self.gating_max_duration_minutes = _GATING_VOC_MAX_DURATION_MINUTES
            self.init_duration_mean = _INIT_DURATION_MEAN_VOC
            self.init_duration_variance = _INIT_DURATION_VARIANCE_VOC
            self.gating_threshold = _GATING_THRESHOLD_VOC

        self.tau_mean_hours = _TAU_MEAN_HOURS
        self.tau_variance_hours = _TAU_VARIANCE_HOURS
        self.sraw_std_initial = _SRAW_STD_INITIAL
        self.index_gain = _INDEX_GAIN

        self.reset()

    def reset(self):
        """Сброс внутреннего состояния алгоритма."""
        self.uptime = 0.0
        self.sraw = 0.0
        self.gas_index = 0.0
        self._init_instances()

    def _init_instances(self):
        """Присвоение начальных значений (соответствует _init_instances в C)."""
        self._mve_initialized = False
        self._mve_mean = 0.0
        self._mve_sraw_offset = 0.0
        self._mve_std = self.sraw_std_initial

        dt_hours = self.sampling_interval / _SEC_IN_HOUR
        self._mve_gamma_mean = ((_MVE_ADDITIONAL_GAMMA_MEAN_SCALING * _MVE_GAMMA_SCALING) * dt_hours) / (self.tau_mean_hours + dt_hours)
        self._mve_gamma_variance = (_MVE_GAMMA_SCALING * dt_hours) / (self.tau_variance_hours + dt_hours)

        if self.algorithm_type == ALGORITHM_TYPE_NOX:
            self._mve_gamma_initial_mean = ((_MVE_ADDITIONAL_GAMMA_MEAN_SCALING * _MVE_GAMMA_SCALING) * self.sampling_interval) / (_TAU_INITIAL_MEAN_NOX + self.sampling_interval)
        else:
            self._mve_gamma_initial_mean = ((_MVE_ADDITIONAL_GAMMA_MEAN_SCALING * _MVE_GAMMA_SCALING) * self.sampling_interval) / (_TAU_INITIAL_MEAN_VOC + self.sampling_interval)

        self._mve_gamma_initial_variance = (_MVE_GAMMA_SCALING * self.sampling_interval) / (_TAU_INITIAL_VARIANCE + self.sampling_interval)

        self._mve_uptime_gamma = 0.0
        self._mve_uptime_gating = 0.0
        self._mve_gating_duration_minutes = 0.0

        # MOX Model init
        self._mox_sraw_std = self._mve_std
        self._mox_sraw_mean = self._mve_mean

        # Sigmoid Scaled init
        if self.algorithm_type == ALGORITHM_TYPE_NOX:
            self._sigmoid_scaled_k = _SIGMOID_K_NOX
            self._sigmoid_scaled_x0 = _SIGMOID_X0_NOX
            self._sigmoid_scaled_offset_default = _NOX_INDEX_OFFSET_DEFAULT
        else:
            self._sigmoid_scaled_k = _SIGMOID_K_VOC
            self._sigmoid_scaled_x0 = _SIGMOID_X0_VOC
            self._sigmoid_scaled_offset_default = _VOC_INDEX_OFFSET_DEFAULT

        # Adaptive Lowpass init
        self._lp_a1 = self.sampling_interval / (_LP_TAU_FAST + self.sampling_interval)
        self._lp_a2 = self.sampling_interval / (_LP_TAU_SLOW + self.sampling_interval)
        self._lp_initialized = False
        self._lp_x1 = 0.0
        self._lp_x2 = 0.0
        self._lp_x3 = 0.0

    # Внутренние математические функции

    @staticmethod
    def _calc_sigmoid(sample: float, k: float, x0: float) -> float:
        """Вычисление сигмоиды с защитой от переполнения exp()."""
        val = k * (sample - x0)
        if val < -50.0:
            return 1.0
        if val > 50.0:
            return 0.0
        return 1.0 / (1.0 + math.exp(val))

    def _adaptive_lowpass_process(self, sample: float) -> float:
        """Адаптивный ФНЧ для сглаживания выходного индекса."""
        if not self._lp_initialized:
            self._lp_x1 = sample
            self._lp_x2 = sample
            self._lp_x3 = sample
            self._lp_initialized = True

        self._lp_x1 = (1.0 - self._lp_a1) * self._lp_x1 + self._lp_a1 * sample
        self._lp_x2 = (1.0 - self._lp_a2) * self._lp_x2 + self._lp_a2 * sample

        abs_delta = abs(self._lp_x1 - self._lp_x2)
        f1 = math.exp(_LP_ALPHA * abs_delta)
        tau_a = (_LP_TAU_SLOW - _LP_TAU_FAST) * f1 + _LP_TAU_FAST
        a3 = self.sampling_interval / (self.sampling_interval + tau_a)

        self._lp_x3 = (1.0 - a3) * self._lp_x3 + a3 * sample
        return self._lp_x3

    def _mox_model_process(self, sraw: float) -> float:
        """Расчет базового отклика модели MOX."""
        if self.algorithm_type == ALGORITHM_TYPE_NOX:
            return ((sraw - self._mox_sraw_mean) / _SRAW_STD_NOX) * self.index_gain
        else:
            return ((sraw - self._mox_sraw_mean) / (-1.0 * (self._mox_sraw_std + _SRAW_STD_BONUS_VOC))) * self.index_gain

    def _sigmoid_scaled_process(self, sample: float) -> float:
        """Нелинейное масштабирование отклика в диапазон индекса."""
        x = self._sigmoid_scaled_k * (sample - self._sigmoid_scaled_x0)
        if x < -50.0:
            return _SIGMOID_L
        if x > 50.0:
            return 0.0

        if sample >= 0.0:
            if self._sigmoid_scaled_offset_default == 1.0:
                shift = (500.0 / 499.0) * (1.0 - self.index_offset)
            else:
                shift = (_SIGMOID_L - 5.0 * self.index_offset) / 4.0
            return ((_SIGMOID_L + shift) / (1.0 + math.exp(x))) - shift
        else:
            return (self.index_offset / self._sigmoid_scaled_offset_default) * (_SIGMOID_L / (1.0 + math.exp(x)))

    def _mve_process(self, sraw: float):
        """Обновление оценки скользящего среднего и дисперсии (Mean Variance Estimator)."""
        if not self._mve_initialized:
            self._mve_initialized = True
            self._mve_sraw_offset = sraw
            self._mve_mean = 0.0
            return

        if self._mve_mean >= 100.0 or self._mve_mean <= -100.0:
            self._mve_sraw_offset += self._mve_mean
            self._mve_mean = 0.0

        sraw_shifted = sraw - self._mve_sraw_offset

        # Лимит для предотвращения переполнения float
        uptime_limit = _MVE_FIX16_MAX - self.sampling_interval
        if self._mve_uptime_gamma < uptime_limit:
            self._mve_uptime_gamma += self.sampling_interval
        if self._mve_uptime_gating < uptime_limit:
            self._mve_uptime_gating += self.sampling_interval

        # Расчет гаммы среднего значения
        sigmoid_gamma_mean = self._calc_sigmoid(self._mve_uptime_gamma, _INIT_TRANSITION_MEAN, self.init_duration_mean)
        gamma_mean = self._mve_gamma_mean + (self._mve_gamma_initial_mean - self._mve_gamma_mean) * sigmoid_gamma_mean

        # Порог для среднего значения
        gating_threshold_mean = self.gating_threshold + (_GATING_THRESHOLD_INITIAL - self.gating_threshold) * \
                                self._calc_sigmoid(self._mve_uptime_gating, _INIT_TRANSITION_MEAN, self.init_duration_mean)

        # Эффективная гамма среднего значения
        sigmoid_gating_mean = self._calc_sigmoid(self.gas_index, _GATING_THRESHOLD_TRANSITION, gating_threshold_mean)
        gamma_mean_effective = sigmoid_gating_mean * gamma_mean

        # Расчет гаммы для дисперсии
        sigmoid_gamma_variance = self._calc_sigmoid(self._mve_uptime_gamma, _INIT_TRANSITION_VARIANCE, self.init_duration_variance)
        gamma_variance = self._mve_gamma_variance + (self._mve_gamma_initial_variance - self._mve_gamma_variance) * (sigmoid_gamma_variance - sigmoid_gamma_mean)

        # Порог для дисперсии
        gating_threshold_variance = self.gating_threshold + (_GATING_THRESHOLD_INITIAL - self.gating_threshold) * \
                                    self._calc_sigmoid(self._mve_uptime_gating, _INIT_TRANSITION_VARIANCE, self.init_duration_variance)

        # Эффективная гамма дисперсии
        sigmoid_gating_variance = self._calc_sigmoid(self.gas_index, _GATING_THRESHOLD_TRANSITION, gating_threshold_variance)
        gamma_variance_effective = sigmoid_gating_variance * gamma_variance

        # Обновление длительности
        self._mve_gating_duration_minutes += (self.sampling_interval / _SEC_IN_MIN) * (((1.0 - sigmoid_gating_mean) * (1.0 + _GATING_MAX_RATIO)) - _GATING_MAX_RATIO)
        if self._mve_gating_duration_minutes < 0.0:
            self._mve_gating_duration_minutes = 0.0
        if self._mve_gating_duration_minutes > self.gating_max_duration_minutes:
            self._mve_uptime_gating = 0.0

        # Обновление среднего значения и стандартного отклонения
        delta_sgp = (sraw_shifted - self._mve_mean) / _MVE_GAMMA_SCALING

        if delta_sgp < 0.0:
            c = self._mve_std - delta_sgp
        else:
            c = self._mve_std + delta_sgp

        additional_scaling = 1.0
        if c > _X2:
            additional_scaling = (c / _X2) ** 2

        # Формула обновления std
        term1 = math.sqrt(additional_scaling * (_MVE_GAMMA_SCALING - gamma_variance_effective))
        term2_inner = (self._mve_std * (self._mve_std / (_MVE_GAMMA_SCALING * additional_scaling))) + \
                      (((gamma_variance_effective * delta_sgp) / additional_scaling) * delta_sgp)
        term2 = math.sqrt(max(0.0, term2_inner))
        self._mve_std = term1 * term2

        # Формула обновления mean
        self._mve_mean += (gamma_mean_effective * delta_sgp) / _MVE_ADDITIONAL_GAMMA_MEAN_SCALING

    # Публичный API

    def process(self, sraw: int) -> int:
        """
        Принимает сырое значение (int) и возвращает индекс (int).
        Соответствует функции GasIndexAlgorithm_process в оригинальном C-коде.
        """
        if self.uptime <= INITIAL_BLACKOUT:
            self.uptime += self.sampling_interval
        else:
            # Ограничение входного сигнала согласно спецификации
            if 0 < sraw < 65000:
                if sraw < self.sraw_minimum + 1:
                    sraw = self.sraw_minimum + 1
                elif sraw > self.sraw_minimum + _X1:
                    sraw = self.sraw_minimum + _X1
                self.sraw = float(sraw - self.sraw_minimum)

            # 1. Расчет индекса через MOX модель и Sigmoid Scaling
            if self.algorithm_type == ALGORITHM_TYPE_VOC or self._mve_initialized:
                self.gas_index = self._mox_model_process(self.sraw)
                self.gas_index = self._sigmoid_scaled_process(self.gas_index)
            else:
                self.gas_index = self.index_offset

            # 2. Адаптивное сглаживание выходного индекса
            self.gas_index = self._adaptive_lowpass_process(self.gas_index)
            if self.gas_index < 0.5:
                self.gas_index = 0.5

            # 3. Обновление оценщика дисперсии (использует уже рассчитанный gas_index для gating)
            if self.sraw > 0.0:
                self._mve_process(self.sraw)
                self._mox_sraw_std = self._mve_std
                self._mox_sraw_mean = self._mve_mean + self._mve_sraw_offset

        # Округление как в оригинале: (int32_t)(mGas_Index + 0.5f)
        return int(self.gas_index + 0.5)

    # для совместимости со старым кодом
    def get_gas_index(self, sraw: int) -> int:
        return self.process(sraw)