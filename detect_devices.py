import os
import subprocess


def main() -> None:
    devices = [f'/dev/{device}' for device in os.listdir('/dev') if 'USB' or 'ACM' in device]
    devices.reverse()
    for device in devices:
        display_name = get_device_name(device)
        if display_name == 'USB2.0-Serial 1a86':
            print(f'export ARDUINO_DEVICE={device}')
        if display_name == 'USB_Serial Teensyduino':
            print(f'export TEENSY_DEVICE={device}')


def get_device_name(device: str) -> str:
    """
    Retrieve a human-readable name for a UNIX device.

    :param device: A device file, e.g. /dev/ttyACM0
    :return: A human-readable representation
    """
    # Retrieve metadata for a specific device
    result = subprocess.run(
        [
            'bash',
            '-c',
            f'udevadm info -p $(udevadm info -q path -n {device})'
        ],
        capture_output=True,
        text=True
    )
    # Other fields might work just as well or even better, but these two have
    # the additional advantage of being human-readable.
    return ' '.join([
        line.split('=')[1]
        for line
        in result.stdout.split('\n')
        if 'ID_VENDOR=' in line or 'ID_MODEL=' in line
    ])


if __name__ == '__main__':
    main()
