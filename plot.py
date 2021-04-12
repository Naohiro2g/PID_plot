import matplotlib.pyplot as plt


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.targetPos = 0.
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.delta_time = 0.1
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0

    def update(self, feedback_value):
        error = self.targetPos - feedback_value
        delta_error = error - self.last_error
        self.PTerm = self.Kp * error
        self.ITerm += error * self.delta_time

        if (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard
        if(self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        self.DTerm = delta_error / self.delta_time
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setTargetPosition(self, targetPos):
        self.targetPos = targetPos


if __name__ == "__main__":
    pid = PID(1.0, 1.45, 0.00015)

    RepeatNum = 100
    feedback = 0
    target_position = []
    target_position.append(0.0)
    for i in range(1, RepeatNum):
        if i < RepeatNum // 2:
            target_position.append(1.0)
        else:
            target_position.append(0.5)

    feedback_list = []

    for i in range(1, RepeatNum):
        pid.update(feedback)
        feedback += pid.output
        pid.setTargetPosition(target_position[i])
        feedback_list.append(feedback)

    plt.title('PID control simulator')
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.plot(feedback_list, label='target')
    plt.plot(target_position, label='feedback')
    plt.ylim(min(target_position) * -0.2, max(target_position) * 1.5)
    plt.legend(loc='lower right')
    plt.show()
