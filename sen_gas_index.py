"""
Порт официального алгоритма Sensirion Gas Index Algorithm (v3.2+) для MicroPython.
A port of the official Sensirion Gas Index Algorithm (v3.2+) for MicroPython.

Модуль предназначен для программной обработки сырых тиков (SRAW_VOC и SRAW_NOx),
получаемых от датчиков качества воздуха Sensirion SGP40 и SGP41, и расчёта
относительных индексов газов в диапазоне от 1 до 500:
  - VOC Index (ALGORITHM_TYPE_VOC): 100 - средний фоновый уровень чистого воздуха;
    < 100 - улучшение качества воздуха; > 100 - рост концентрации ЛОС.
  - NOx Index (ALGORITHM_TYPE_NOX): 1 - чистый воздух без оксидов азота;
    > 1 - появление оксидов азота (продукты горения, выхлопы).

Особенности работы:
  - Требуется строгий постоянный интервал опроса (по умолчанию 1.0 с).
  - В течение первых 45 секунд после запуска (период начального прогрева)
    метод get_gas_index() возвращает 0.
"""


import math
from micropython import const
from sensor_pack_2.base_sensor import check_value

# Общедоступные постоянные типа алгоритма
ALGORITHM_TYPE_VOC = const(0)
ALGORITHM_TYPE_NOX = const(1)
# Внутренние целочисленные постоянные
_SRAW_MIN_VOC = const(20000)
_SRAW_MIN_NOX = const(10000)
# Внутренние константы алгоритма Sensirion v3.2+ (float)
_INITIAL_BLACKOUT = 45.0
_INDEX_GAIN = 230.0
_SRAW_STD_INITIAL = 50.0
_SRAW_STD_BONUS_VOC = 220.0
_SRAW_STD_NOX = 2000.0
_TAU_MEAN_HOURS = 12.0
_TAU_VARIANCE_HOURS = 12.0
#
_GATING_MAX_DURATION_MINUTES_VOC = 180.0
_GATING_MAX_DURATION_MINUTES_NOX = 720.0
_GATING_THRESHOLD_VOC = 340.0
_GATING_THRESHOLD_NOX = 30.0
_GATING_THRESHOLD_INITIAL = 510.0
_GATING_THRESHOLD_TRANSITION = 0.09
#
_VOC_INDEX_OFFSET_DEFAULT = 100.0
_NOX_INDEX_OFFSET_DEFAULT = 1.0
#
_LP_TAU_FAST = 20.0
_LP_TAU_SLOW = 500.0
_LP_ALPHA = -0.2
#
_GAS_INDEX_MIN = 1.0
_GAS_INDEX_MAX = 500.0
_SRAW_STD_MIN = _GAS_INDEX_MIN
_SEC_IN_HOUR = const(3600)
# порог защиты от переполнения разрядности (fix16_t overflow) при вычислении квадрата стандартного отклонения
_MVE_STD_SCALING_THRESHOLD = const(1440)
_ONE_HUNDRED = const(100)
#
_INIT_DURATION_MEAN_VOC = _SEC_IN_HOUR * 0.75
_INIT_DURATION_MEAN_NOX = _SEC_IN_HOUR * 4.75
_INIT_DURATION_VARIANCE_VOC = _SEC_IN_HOUR * 1.45
_INIT_DURATION_VARIANCE_NOX = _SEC_IN_HOUR * 5.70
_INIT_TRANSITION_MEAN = 0.01
_INIT_TRANSITION_VARIANCE = 0.014
#
_MVE_GAMMA_SCALING = 64.0
_MVE_ADDITIONAL_GAMMA_MEAN_SCALING = 8.0


def _sigmoid(x: float, l: float, k: float, x0: float) -> float:
    """Вычисление сигмоиды с защитой от переполнения math.exp()."""
    val = k * (x - x0)
    if val > 50.0:
        return 0.0
    if val < -50.0:
        return l
    return l / (1.0 + math.exp(val))

class GasIndexAlgorithm:
    """Порт официального алгоритма Sensirion Gas Index Algorithm (VOC / NOx) для MicroPython."""

    def __init__(self, algorithm_type: int = ALGORITHM_TYPE_VOC, sampling_interval: float=1.0):
        check_value(algorithm_type, range(2), f"Invalid algorithm type value: {algorithm_type}")
        self.algorithm_type = algorithm_type
        self.sampling_interval = float(sampling_interval)
        #
        self.uptime = 0.0
        self.sraw = 0.0
        self.gas_index = 0
        #
        self._lp_a1 = 0.0
        self._lp_a2 = 0.0
        self._lp_initialized = False
        self._lp_x1 = 0.0
        self._lp_x2 = 0.0
        self._lp_x3 = 0.0

        # Состояние оценки среднего и дисперсии
        self._mve_initialized = False
        self._mve_mean = 0.0
        self._mve_sraw_offset = 0.0
        self._mve_std = _SRAW_STD_INITIAL
        #
        self._mve_uptime_gating = 0.0
        self._mve_gating_duration_minutes = 0.0
        #
        self._mve_gamma_mean = 0.0
        self._mve_gamma_variance = 0.0
        # ALGORITHM_TYPE_VOC
        self.index_offset = _VOC_INDEX_OFFSET_DEFAULT
        self.sraw_minimum = _SRAW_MIN_VOC
        self.gating_max_duration_minutes = _GATING_MAX_DURATION_MINUTES_VOC
        self.init_duration_mean = _INIT_DURATION_MEAN_VOC
        self.init_duration_variance = _INIT_DURATION_VARIANCE_VOC
        self.gating_threshold = _GATING_THRESHOLD_VOC
        self._sraw_std_bonus = _SRAW_STD_BONUS_VOC

        if algorithm_type == ALGORITHM_TYPE_NOX:
            self.index_offset = _NOX_INDEX_OFFSET_DEFAULT
            self.sraw_minimum = _SRAW_MIN_NOX
            self.gating_max_duration_minutes = _GATING_MAX_DURATION_MINUTES_NOX
            self.init_duration_mean = _INIT_DURATION_MEAN_NOX
            self.init_duration_variance = _INIT_DURATION_VARIANCE_NOX
            self.gating_threshold = _GATING_THRESHOLD_NOX
            self._sraw_std_bonus = 0.0

        self.reset()

    def reset(self):
        """Сброс внутреннего состояния фильтров и оценок алгоритма."""
        self.uptime = 0.0
        self.sraw = 0.0
        self.gas_index = 0

        # Состояние адаптивного фильтра низких частот
        self._lp_a1 = self.sampling_interval / (_LP_TAU_FAST + self.sampling_interval)
        self._lp_a2 = self.sampling_interval / (_LP_TAU_SLOW + self.sampling_interval)
        self._lp_initialized = False
        self._lp_x1 = 0.0
        self._lp_x2 = 0.0
        self._lp_x3 = 0.0

        # Состояние оценки среднего значения и дисперсии
        self._mve_initialized = False
        self._mve_mean = 0.0
        self._mve_sraw_offset = 0.0
        self._mve_std = _SRAW_STD_INITIAL

        dt_hours = self.sampling_interval / _SEC_IN_HOUR
        self._mve_gamma_mean = (
            (_MVE_ADDITIONAL_GAMMA_MEAN_SCALING * _MVE_GAMMA_SCALING)
            * dt_hours
            / (_TAU_MEAN_HOURS + dt_hours)
        )
        self._mve_gamma_variance = (
            _MVE_GAMMA_SCALING
            * dt_hours
            / (_TAU_VARIANCE_HOURS + dt_hours)
        )
        self._mve_uptime_gating = 0.0
        self._mve_gating_duration_minutes = 0.0

    def _adaptive_lowpass(self, sample: float) -> float:
        """Адаптивный ФНЧ для сглаживания резких выбросов сырого сигнала."""
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

    def _mean_variance_estimator(self, sraw: float, gating_probability: float) -> None:
        """Обновляет оценку скользящего среднего и стандартного отклонения базового уровня."""
        if not self._mve_initialized:
            self._mve_initialized = True
            self._mve_sraw_offset = sraw
            self._mve_mean = 0.0
            return

        if self._mve_mean >= _ONE_HUNDRED or self._mve_mean <= -_ONE_HUNDRED:
            self._mve_sraw_offset += self._mve_mean
            self._mve_mean = 0.0

        sraw_shifted = sraw - self._mve_sraw_offset

        sigmoid_gamma_mean = _sigmoid(
            self._mve_uptime_gating,
            1.0,
            _INIT_TRANSITION_MEAN,
            self.init_duration_mean,
        )
        gamma_mean = self._mve_gamma_mean + (
            (_MVE_GAMMA_SCALING - self._mve_gamma_mean) * sigmoid_gamma_mean
        )

        gating_threshold_mean = self.gating_threshold + (
            (_GATING_THRESHOLD_INITIAL - self.gating_threshold)
            * _sigmoid(
                self._mve_uptime_gating,
                1.0,
                _GATING_THRESHOLD_TRANSITION,
                self.init_duration_mean,
            )
        )
        sigmoid_gating_mean = _sigmoid(gating_probability, 1.0, 1.0, gating_threshold_mean)
        gamma_mean_effective = (sigmoid_gating_mean * gamma_mean) / _MVE_GAMMA_SCALING

        sigmoid_gamma_var = _sigmoid(
            self._mve_uptime_gating,
            1.0,
            _INIT_TRANSITION_VARIANCE,
            self.init_duration_variance,
        )
        gamma_var = self._mve_gamma_variance + (
            (_MVE_GAMMA_SCALING - self._mve_gamma_variance)
            * (sigmoid_gamma_var - sigmoid_gamma_mean)
        )
        gamma_var_effective = (sigmoid_gating_mean * gamma_var) / _MVE_GAMMA_SCALING

        if gating_probability > gating_threshold_mean:
            self._mve_gating_duration_minutes += self.sampling_interval / 60.0
        else:
            self._mve_gating_duration_minutes = 0.0

        self._mve_uptime_gating += self.sampling_interval
        if 0.0 < self.gating_max_duration_minutes < self._mve_gating_duration_minutes:
            self._mve_uptime_gating = 0.0

        delta_sgp = (sraw_shifted - self._mve_mean) / _MVE_GAMMA_SCALING
        c = self._mve_std + abs(delta_sgp)
        additional_scaling = 1.0 if c <= _MVE_STD_SCALING_THRESHOLD else (c / _MVE_STD_SCALING_THRESHOLD) ** 2

        self._mve_std = math.sqrt(
            max(
                0.0,
                additional_scaling
                * (1.0 - gamma_var_effective)
                * ((self._mve_std * (self._mve_std / additional_scaling)) + (gamma_var_effective * delta_sgp * delta_sgp)),
            )
        )
        self._mve_mean += gamma_mean_effective * delta_sgp

    def get_gas_index(self, sraw: int) -> int:
        """
        Принимает сырое значение sraw (int) и возвращает газовый индекс (_GAS_INDEX_MIN.._GAS_INDEX_MAX).
        В первые _INITIAL_BLACKOUT секунд прогрева возвращает 0.
        """
        if sraw <= 0:
            return 0

        if self.uptime <= _INITIAL_BLACKOUT:
            self.uptime += self.sampling_interval
            if self.uptime > _INITIAL_BLACKOUT:
                # в момент завершения прогрева сброс фильтров,
                # чтобы начальный разогрев от 20500 до 27500+ не сместил базовую линию!
                self._mve_initialized = False
                self._lp_initialized = False

        sraw_val = float(sraw) - self.sraw_minimum

        if self.uptime <= _INITIAL_BLACKOUT:
            self.sraw = self._adaptive_lowpass(sraw_val)
            self.gas_index = 0
            return 0

        # обновление адаптивного ФНЧ (на 1+_INITIAL_BLACKOUT секунде он инициализируется текущим sraw_val)
        self.sraw = self._adaptive_lowpass(sraw_val)

        # Обновление оценки среднего и дисперсии по сглаженному значению
        self._mean_variance_estimator(self.sraw, float(self.gas_index))

        mve_mean = self._mve_mean + self._mve_sraw_offset
        mve_std = _SRAW_STD_NOX if self.algorithm_type == ALGORITHM_TYPE_NOX else (self._mve_std + self._sraw_std_bonus)

        if mve_std < _SRAW_STD_MIN:
            mve_std = _SRAW_STD_MIN

        if self.algorithm_type == ALGORITHM_TYPE_NOX:
            gas_index_f = ((self.sraw - mve_mean) / mve_std) * _INDEX_GAIN + self.index_offset
        else:
            gas_index_f = ((mve_mean - self.sraw) / mve_std) * _INDEX_GAIN + self.index_offset

        if gas_index_f < _GAS_INDEX_MIN:
            gas_index_f = _GAS_INDEX_MIN
        elif gas_index_f > _GAS_INDEX_MAX:
            gas_index_f = _GAS_INDEX_MAX

        self.gas_index = int(round(gas_index_f))
        return self.gas_index
