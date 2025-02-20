import trossen_slate as slate
import time
import asyncio
import threading

def test_slate():
    slate_driver = slate.TrossenSlate()

    slate_driver.init_base()
    print(f"CURRENT CHARGE: {slate_driver.get_charge()}")
    print(f"CURRENT CURRENT: {slate_driver.get_current()}")
    print(f"CURRENT POSE: {slate_driver.get_pose()}")
    print(f"CURRENT VEL: {slate_driver.get_vel()}")
    print(f"CURRENT VOLTAGE: {slate_driver.get_voltage()}")
    # light_state = slate.LightState()
    # light_state.color = slate.LightState.Color.YELLOW
    colors = [slate.LightState.RED, slate.LightState.GREEN, slate.LightState.BLUE, slate.LightState.YELLOW, slate.LightState.CYAN, slate.LightState.PURPLE, slate.LightState.WHITE]
    # print(f"CURRENT LIGHT STATE: {slate_driver.get_light_state()}")

    def update_values(slate_driver, frequency=1, duration=10):
        start_time = time.time()
        log_data = slate.ChassisData()
        while time.time() - start_time < duration:
            slate_driver.update_state()
            slate_driver.read(log_data)
            # slate_driver.set_light_state(colors[int(time.time()) % len(colors)])
            print(f"UPDATED POSE: {log_data.vel_z}, {log_data.vel_x}")
            # time.sleep(1 / frequency)

    def test_commands(slate_driver, linear_vel, angular_vel, duration=5):
        log_data = slate.ChassisData()
        start_time = time.time()
        while time.time() - start_time < duration:
            slate_driver.set_cmd_vel(linear_vel, angular_vel)
            time.sleep(0.1)

    update_thread = threading.Thread(target=update_values, args=(slate_driver, 30, 10))
    update_thread.start()

    test_commands(slate_driver, 0.0, 0.0, 2)
    # test_commands(slate_driver, -0.1, 0.0, 2)

    update_thread.join()  # Wait for the update thread to finish


test_slate()