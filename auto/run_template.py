import matplotlib
matplotlib.use('qtagg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Slider
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

vel_data = []
vel_data_wo_ff = []
error_data = []
error_data_wo_ff = []
time_data = []
angle_data = []
axs = []

# K_P = .07
# K_I = .00007
# K_D = 0

STEPS = 2000

def draw_graph(ax, x, y, label, linestyle, title):
    ax.set_title(title)
    for i in range(len(y)):
        axs.append(ax.plot(x, y[i], label=label[i], linestyle=linestyle[i]))
    ax.legend(loc="upper left")

def run_sim(val):
    ax1.clear()
    ax2.clear()

    car = make_car(desired_v=40, dt=0.1)
    carNF = make_car(desired_v=40, dt=0.1)

    vel_data.clear()
    vel_data_wo_ff.clear()
    error_data.clear()
    error_data_wo_ff.clear()
    time_data.clear()
    angle_data.clear()

    K_P = K_P_slider.val
    K_I = K_I_slider.val
    K_D = K_D_slider.val

    for i in range(STEPS):
        desired_acc, error = calculate_desired_acceleration(car, K_P, K_I, K_D, True)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(car, desired_throttle)

        error_data.append(error)

        desired_acc, error = calculate_desired_acceleration(carNF, K_P, K_I, K_D, False)
        desired_throttle = acceleration_to_throttle_percentage(desired_acc)
        update(carNF, desired_throttle)

        vel_data.append(car["v"])
        vel_data_wo_ff.append(carNF["v"])
        error_data_wo_ff.append(error)
        time_data.append(car["t"])
        angle_data.append(car["angle"])
    draw_graph(ax1, time_data, [vel_data, error_data, angle_data], ["velocity", "error", "angle"], ["solid", "solid", "dashed"], "With Feedforward")
    draw_graph(ax2, time_data, [vel_data_wo_ff, error_data_wo_ff, angle_data], ["velocity", "error", "angle"], ["solid", "solid", "dashed"], "Without Feedforward")


fig = plt.figure(layout="constrained")
gs = GridSpec(5, 2, figure=fig)
ax1 = fig.add_subplot(5, 2, (1, 2))
ax2 = fig.add_subplot(5, 2, (3, 4))
ax3 = fig.add_subplot(5, 2, (5, 6))
ax4 = fig.add_subplot(5, 2, (7, 8))
ax5 = fig.add_subplot(5, 2, (9, 10))
K_P_slider = Slider(ax=ax3, label="K_P", valmin=0, valmax=1, valinit=.01, valfmt="%.4f")
K_I_slider = Slider(ax=ax4, label="K_I", valmin=0, valmax=1, valinit=.01, valfmt="%.4f")
K_D_slider = Slider(ax=ax5, label="K_D", valmin=0, valmax=1, valinit=.01, valfmt="%.4f")
K_P_slider.on_changed(run_sim)
K_I_slider.on_changed(run_sim)
K_D_slider.on_changed(run_sim)
plt.suptitle("PID Demo")
run_sim(0)
plt.show()
