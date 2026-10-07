import matplotlib
matplotlib.use('qtagg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Slider
from matplotlib.widgets import CheckButtons

def init_graph(update_data: function) -> None:
    global ax1
    global ax2
    global K_P_slider
    global K_I_slider
    global K_D_slider
    global fig
    global control_mode
    fig = plt.figure(layout="constrained")
    GridSpec(6, 2, figure=fig)
    ax1 = fig.add_subplot(6, 2, (1, 2))
    ax2 = fig.add_subplot(6, 2, (3, 4))
    ax2.set_title("Without Feedforward")
    ax3 = fig.add_subplot(6, 2, (5, 6))
    ax4 = fig.add_subplot(6, 2, (7, 8))
    ax5 = fig.add_subplot(6, 2, (9, 10))
    bax = plt.axes([.01, .01, .2, .05])
    control_mode = CheckButtons(bax, ["Position Control"], [False])
    # control_mode.on_clicked()
    K_P_slider = Slider(ax=ax3, label="K_P", valmin=0, valmax=.1, valinit=.001, valfmt="%.4f")
    K_I_slider = Slider(ax=ax4, label="K_I", valmin=0, valmax=.01, valinit=.00001, valfmt="%.6f")
    K_D_slider = Slider(ax=ax5, label="K_D", valmin=0, valmax=1, valinit=.01, valfmt="%.4f")
    K_P_slider.on_changed(update_data)
    K_I_slider.on_changed(update_data)
    K_D_slider.on_changed(update_data)
    plt.suptitle("PID Demo")

def get_slider_values() -> tuple:
    return (K_P_slider.val, K_I_slider.val, K_D_slider.val);

# def get_control_mode_request() -> bool:
#     return 

def draw_all(with_feedforward_container, without_feedforward_container):
    draw_graph(ax1, with_feedforward_container["time_data"], [with_feedforward_container["vel_data"], with_feedforward_container["error_data"], with_feedforward_container["angle_data"]], ["velocity", "error", "angle"], ["solid", "solid", "dashed"], "With Feedforward")
    draw_graph(ax2, without_feedforward_container["time_data"], [without_feedforward_container["vel_data"], without_feedforward_container["error_data"], without_feedforward_container["angle_data"]], ["velocity", "error", "angle"], ["solid", "solid", "dashed"], "Without Feedforward")

def draw_graph(ax, x, y, label, linestyle, title):
    ax.clear()
    ax.set_title(title)
    for i in range(len(y)):
        ax.plot(x, y[i], label=label[i], linestyle=linestyle[i])
    ax.legend(loc="upper left")