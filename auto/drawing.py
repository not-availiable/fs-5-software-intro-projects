import matplotlib
matplotlib.use('qtagg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Slider
from matplotlib.widgets import CheckButtons
from matplotlib.axes import Axes

def init_graph(update_data: function) -> None:
    """
    Sets up the matplotlib graph

    Inputs:
    update_data: the function that should be called whenever a ui element makes a change that requires the sim to be re-run and the graphs to be re-drawn
    """
    global feedforward_ax
    global no_feedforward_ax
    global K_P_slider
    global K_I_slider
    global K_D_slider
    global vel_setpoint_slider
    global pos_setpoint_slider
    global control_mode_button

    num_axes = 7
    num_graphs = 2
    #makes graphs x times bigger than the sliders
    graph_to_slider_ratio = 4
    #reserves the space on the grid for the graphs and sliders, required_space is +1ed to reserve room for the position button
    #which looks terrible when on the standard grid
    graph_space = num_graphs*graph_to_slider_ratio
    required_space = graph_space+num_axes-num_graphs+1
    fig = plt.figure(layout="constrained")
    graphs = [fig.add_subplot(required_space, 2, (2*graph_to_slider_ratio*i+1, 2*graph_to_slider_ratio*i+2*graph_to_slider_ratio)) for i in range(num_graphs)]
    sliders = [fig.add_subplot(required_space, 2, (graph_space*2+2*i+1, graph_space*2+2*i+2)) for i in range(num_axes-num_graphs)]
    feedforward_ax = graphs[0]
    no_feedforward_ax = graphs[1]
    kp_ax = sliders[0]
    ki_ax = sliders[1]
    kd_ax = sliders[2]
    vel_ax = sliders[3]
    pos_ax = sliders[4]
    position_mode_toggle_ax = plt.axes([.01, .01, .2, .05])
    control_mode_button = CheckButtons(position_mode_toggle_ax, ["Position Control"], [False])
    control_mode_button.on_clicked(update_data)
    K_P_slider = Slider(ax=kp_ax, label="K_P", valmin=0, valmax=1, valinit=.001, valfmt="%.4f")
    K_I_slider = Slider(ax=ki_ax, label="K_I", valmin=0, valmax=.01, valinit=.00001, valfmt="%.6f")
    K_D_slider = Slider(ax=kd_ax, label="K_D", valmin=0, valmax=.5, valinit=.01, valfmt="%.4f")
    vel_setpoint_slider = Slider(ax=vel_ax, label="vel_setpoint", valmin=0, valmax=100, valinit=10, valfmt="%.2f")
    pos_setpoint_slider = Slider(ax=pos_ax, label="pos_setpoint", valmin=10, valmax=2000, valinit=400, valfmt="%.2f")
    K_P_slider.on_changed(update_data)
    K_I_slider.on_changed(update_data)
    K_D_slider.on_changed(update_data)
    vel_setpoint_slider.on_changed(update_data)
    pos_setpoint_slider.on_changed(update_data)
    plt.suptitle("PID Demo")

def get_control_mode() -> bool:
    return control_mode_button.get_status()[0]

def get_slider_values() -> tuple:
    return (K_P_slider.val, K_I_slider.val, K_D_slider.val, vel_setpoint_slider.val, pos_setpoint_slider.val)

def draw_all(with_feedforward_container: dict, without_feedforward_container: dict) -> None:
    """
    Draws the graphs for both cars
    """
    draw_graph(feedforward_ax, with_feedforward_container["time_data"], [with_feedforward_container["process_data"], with_feedforward_container["error_data"], with_feedforward_container["angle_data"], with_feedforward_container["process_setpoint"]], ["process", "error", "angle", "setpoint"], ["solid", "solid", "dotted", "dashed"], "With Feedforward")
    draw_graph(no_feedforward_ax, without_feedforward_container["time_data"], [without_feedforward_container["process_data"], without_feedforward_container["error_data"], without_feedforward_container["angle_data"], without_feedforward_container["process_setpoint"]], ["process", "error", "angle", "setpoint"], ["solid", "solid", "dotted", "dashed"], "Without Feedforward")

def draw_graph(ax: Axes, x: list, y: list, label: list, linestyle: list, title: str) -> None:
    """
    Inputs:
    ax: which axis to draw the grpah on
    x: a single list representign time
    y: a list of lists and constants, the lists get treated as f(x), the constants get treated as horizontal lines (ie: for displaying the setpoint)
    label: a list of the names for each y
    linestyle: a list of the linestyles for each y
    title: the title displayed above the graph
    """
    ax.clear()
    ax.set_title(title)
    for i in range(len(y)):
        if isinstance(y[i], list):
            ax.plot(x, y[i], label=label[i], linestyle=linestyle[i])
        else:
            ax.axhline(y[i], label=label[i], linestyle=linestyle[i])
    ax.legend(loc="upper left")
    plt.draw()