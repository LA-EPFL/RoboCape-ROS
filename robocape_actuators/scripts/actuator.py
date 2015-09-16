from __future__ import division
import mraa

class Actuator(object):
    def __init__(self, board_pin, period_us, duty_min, duty_max, output_min, output_max):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = duty_max - duty_min

        self.output_min = output_min
        self.output_max = output_max
        self.output_span = output_max - output_min

        self.dev = mraa.Pwm(board_pin)
        self.dev.period_us(period_us)
        self.dev.enable(True)
        self._output = self.output_min

    @property
    def output(self):
        duty = self.dev.read()
        self._output = self.output_min + self.output_span * (duty - self.duty_min) / self.duty_span

        return self._output

    @output.setter
    def output(self, new_output):
        if new_output < self.output_min:
            new_output = self.output_min
        elif new_output > self.output_max:
            new_output = self.output_max

        self._output = new_output
        duty = self.duty_min + self.duty_span * (new_output - self.output_min) / self.output_span

        self.dev.write(duty)
