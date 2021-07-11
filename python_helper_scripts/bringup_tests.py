import sys
import glob
import serial
from serial.serialutil import SerialBase
import serial.tools.list_ports
from rich.console import Console
from rich.table import Table
from rich.prompt import IntPrompt

console = Console()

def get_serial_ports():
    ports = serial.tools.list_ports.comports()
    table = Table(title="Attached serial devices")

    table.add_column("", justify="right", style="cyan", no_wrap=True)
    table.add_column("Port ID", justify="right", style="cyan", no_wrap=True)
    table.add_column("Description", style="magenta")
    table.add_column("Manufacturer", style="magenta")
    table.add_column("Hardware ID", justify="right", style="green")
    
    count = 1
    for p in (ports):
        table.add_row(str(count),p.device, p.description, p.manufacturer, p.hwid)
        count = count + 1
        #console.print("{}: {} [{}]".format(port, desc, hwid))
    return table,len(ports),ports

def serial_port_selector_query():
    port = None
    t,i,ports = get_serial_ports()
    console.print(t)
    selected_id = IntPrompt.ask("Select the device [1-{}, 0 to refresh]".format(i))
    if selected_id > i:
        console.print("Invalid selection...")
        selected_id = 0
    if selected_id != 0:
        port = ports[selected_id-1].device
    return selected_id,port

def serial_port_selector():
    selected_id = 0    
    while selected_id == 0:
        selected_id,port = serial_port_selector_query()
    try:
        ser = serial.Serial(port=port)
        ser.close()
    except:
        console.print("Could not connect to the port. Is it in use?")
        return serial_port_selector()
    return port

def identity_check(ser):
    console.print("Checking board ID:")
    ser.read_all()
    ser.write('*IDN?\n'.encode('utf-8'))
    console.print("   *IDN?   ")
    console.print(ser.readline().decode('utf-8').rstrip())

def meq_single_module_bringup_sequence(ser):
    identity_check(ser)

if __name__ == "__main__":
    console.rule("Starting main")
    port = serial_port_selector()
    console.print("Connecting to device on port {}".format(port))
    ser = serial.Serial(port=port)
    console.rule("Running device bringup test")
    meq_single_module_bringup_sequence(ser)
    