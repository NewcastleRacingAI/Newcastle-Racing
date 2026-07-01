from nrai_perception.node import handle_simulator
from nrai_pathplanning.node import main as handle_pathplanning
from nrai_control.node import main as handle_control
from src.nrai_carmaker.nrai_carmaker_server import main as main_simulator
from multiprocessing import Process, Queue
import argparse
import logging
import subprocess
import os

class Args(argparse.Namespace):
    logger_format: str = "%(asctime)s {%(processName)s} [%(levelname)s] %(filename)s:%(lineno)d => %(message)s"
    verbosity: int = logging.INFO
    zed: bool = False
    camera_topic: str = "/nrai/camera"
    planning_topic: str = "/nrai/planning"
    control_topic: str = "/nrai/control"
    simulator_host: str = "127.0.0.1"
    simulator_bind: str = "0.0.0.0"
    simulator_port: int = 8000
    topics: dict[str, Queue] = {}
    simulator_exe: str = "./src/nrai_carmaker/sensor_client.exe"
    carmaker_host: str = "127.0.0.1"
    carmaker_ports: list[str] = [2210, 2211]



def parse(args: list[str] | None = None):
    default_args = Args()
    parser = argparse.ArgumentParser("nrai_perception")
    parser.add_argument("-z", "--zed", action="store_true", default=default_args.zed, help="Whether we are using the zed camera or a CarMaker simulation")
    parser.add_argument("-ct", "--camera-topic", type=str, default=default_args.camera_topic, help="Topic simulator => perception. Used for both RGB and Depth camera from CarMaker")
    parser.add_argument("-pt", "--planning-topic", type=str, default=default_args.planning_topic, help="Topic perception => planning")
    parser.add_argument("-crt", "--control-topic", type=str, default=default_args.control_topic, help="Topic planning => control")
    parser.add_argument("-sh", "--simulator-host", type=str, default=default_args.simulator_host, help="Ip of the machine the simulator node is running on")
    parser.add_argument("-sb", "--simulator-bind", type=str, default=default_args.simulator_bind, help="Ip used to bind the simulator node server")
    parser.add_argument("-sp", "--simulator-port", type=int, default=default_args.simulator_port, help="Port the simulator node is listening on")
    parser.add_argument("-v", "--verbosity", type=int, default=default_args.verbosity, choices=[10, 20, 30, 40, 50], help="Level of verbosity. 10 => Debug ... 50 => Critical")
    parser.add_argument("-se", "--simulator-exe", type=str, default=default_args.simulator_exe, help="Path to the exe file that captures simulated data from CarMaker and sends it to the simulator node")
    parser.add_argument("-ch", "--carmaker-host", type=str, default=default_args.carmaker_host, help="Ip of the CarMaker camera RSI")
    parser.add_argument("-cp", "--carmaker-ports", type=str, nargs="*", default=default_args.carmaker_ports, help="Ports corresponding to all the CarMaker sensors")
    return parser.parse_args(args, namespace=default_args)

def main(sys_args: list[str] | None = None):
    args = parse(sys_args)

    logging.basicConfig(format=args.logger_format, level=args.verbosity)

    args.topics = {args.camera_topic: Queue(), args.planning_topic: Queue(), args.control_topic: Queue()}

    # NRAI nodes
    control = Process(name="nrai_control", target=handle_control, args=[args], daemon=True)
    pathplanning = Process(name="nrai_pathplanning", target=handle_pathplanning, args=[args], daemon=True)
    perception = Process(name="nrai_perception", target=handle_simulator, args=[args], daemon=True)
    simulator = Process(name="nrai_simulator", target=main_simulator, args=[args], daemon=True)

    control.start()
    pathplanning.start()
    perception.start()
    simulator.start()

    # CarMaker simulated sensors
    carmaker_exes = []
    if args.simulator_exe and args.carmaker_host:
        assert os.path.exists(args.simulator_exe)
        carmaker_exes = [subprocess.Popen([args.simulator_exe, '-p', str(port), '-d', args.simulator_host, "-x", str(args.simulator_port)]) for port in args.carmaker_ports]

    try:
        perception.join(), simulator.join(), control.join(), pathplanning.join()
    except KeyboardInterrupt:
        logging.getLogger().info("All nodes are closed")

    for carmaker_exe in carmaker_exes:
        carmaker_exe.kill()

if __name__ == "__main__":
    main()
