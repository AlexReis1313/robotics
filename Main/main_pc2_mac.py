import multiprocessing
from communication_server import start_server, define_constants
from graph_interface import main_graphics

def receive_robot_data(info_computer_share):
    print('coms')
    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, info_computer_share)

def GUI_loop(info_computer_share):
    print('GUI')
    main_graphics(info_computer_share)

def main():
    info_computer_share = multiprocessing.Manager().dict(
        {'state': -1, 'last_bisturi_pos': [0, 0, 0, 0, 0], 'cutting_plan': [0, 0], 'coliding': False}
    )
  
    graphical_interface_process = multiprocessing.Process(target=GUI_loop, args=(info_computer_share,))
    receive_data_server_process = multiprocessing.Process(target=receive_robot_data, args=(info_computer_share,))


    graphical_interface_process.start()
    receive_data_server_process.start()

    graphical_interface_process.join()
    receive_data_server_process.join()

if __name__ == "__main__":
    main()
