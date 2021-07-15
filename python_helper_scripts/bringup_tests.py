import numpy as np
from asciiplot import asciiize
import serial
import serial.tools.list_ports
from rich.console import Console
from rich.table import Table
from rich.prompt import IntPrompt
from rich.text import Text

console = Console()
error_console = Console(stderr=True, style="bold red")

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

def identity_check(ser,verbose=False):
    console.print("Checking board ID... ",end="")
    ser.read_all() # clear the serial buffers
    data_sent = "*IDN?" # ask for the ID
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    pass_result = "Fail"
    if data_received == "OptoTech,AFC.10.07.2021,LaserDriver,1-channel":
        pass_result = "Pass"
        
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def check_i2c_bus(ser,verbose=False):
    console.print("Checking I2C bus... ",end="")
    ser.read_all() # clear the serial buffers
    data_sent = "SYST:I2C?" # ask for the i2c bus scan
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    pass_result = "Fail"
    if data_received == "0x18,":
        pass_result = "Pass"
        
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def set_pac_addr(ser,verbose=False):
    console.print("Setting PAC sensor address... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:PACADDR 1,24" # set the PAC address
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:PACADDR? 1" # get the PAC address
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_received == "24":
        pass_result = "Pass"
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def set_serial(ser,verbose=False):
    input_value = console.input("Enter the laser serial number (default = 0): ")
    serial_num = 0
    try:
        serial_num = int(input_value)
    except:
        serial_num = 0
    console.print("Setting laser serial number... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:SERIAL 1,{}".format(serial_num) # set the PAC address
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:SERIAL? 1" # get the PAC address
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_received == str(serial_num):
        pass_result = "Pass"
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def set_wavelength(ser,verbose=False):
    input_value = console.input("Enter the laser wavelength (default = 0): ")
    serial_num = 0
    try:
        serial_num = int(input_value)
    except:
        serial_num = 0
    console.print("Setting laser wavelength... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:WAV 1,{}".format(serial_num) # set the wavelength
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:WAV? 1" # get the wavelength
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_received == str(serial_num):
        pass_result = "Pass"
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def set_pmax(ser,verbose=False):
    input_value = console.input("Enter the lasers maximum power in mW (default = 60): ")
    serial_num = 0
    try:
        serial_num = float(input_value)
    except:
        serial_num = 0
    console.print("Setting lasers maximum power in mW... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:PMAX 1,{}".format(serial_num) # set the max power
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:PMAX? 1" # get the max power
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_received == str(serial_num):
        pass_result = "Pass"
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def save_settings(ser,verbose=False):
    console.print("Saving settings to memory... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "*SAV" # save
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_received == "-5":
        pass_result = "Pass"
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def restart_device(ser,verbose=False):
    console.print("Restarting laser... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    port = ser.port
    ser.read_all() # clear the serial buffers
    data_sent = "*RST" # restart
    grid.add_row("Data sent:", " ", data_sent)
    try:
        ser.close()
    except:
        None
    ser = serial.Serial(port=port,timeout=0.3)
    ser.read_all() # clear the serial buffers
    data_sent = "*IDN?" # ask for the ID
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    pass_result = "Fail"
    if data_received == "OptoTech,AFC.10.07.2021,LaserDriver,1-channel":
        pass_result = "Pass"
        
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def test_temperature(ser,verbose=False):
    console.print("Testing the temperature sensor... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "SYST:TEMP?" # get temperature sensor info
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "SYST:TEMP?" # get temperature sensor info
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_float > 5:
        pass_result = "Pass"
        console.print("(value = {})... ".format(data_float),end="")
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def test_voltage(ser,verbose=False):
    console.print("Testing the voltage sensor... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:VOLT? 1" # get laser voltage
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if data_float > 4:
        pass_result = "Pass"
        console.print("(value = {})... ".format(data_float),end="")
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def test_current(ser,verbose=False):
    console.print("Testing the current sensor... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:CURR? 1" # get laser current
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    pass_result = "Fail"
    if (data_float > 1) and (data_float < 20) :
        pass_result = "Pass"
        console.print("(value = {})... ".format(data_float),end="")
        text = Text("[  OK  ]")
        text.stylize("bold green", 0, 8)
        console.print(text)
    else:
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def check_current_pre_trigger(ser,grid):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:CURR? 1" # get laser current
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float > 1) and (data_float < 20) :
        return True
    else:
        return False
    
def check_power_set_mid_range(ser,grid,val=128):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:POW 1,{}".format(val) # set power mid range
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:POW? 1" # get power
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float == val) :
        return True
    else:
        return False

def check_laser_state_on(ser,grid):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:STAT 1,1" # set laser on
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:STAT? 1" # get state
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = int(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float == 1) :
        return True
    else:
        return False

def check_current_while_triggered(ser,grid):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:CURR? 1" # get laser current
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float > 10):
        return True, data_float
    else:
        return False, data_float

def check_laser_state_off(ser,grid):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:STAT 1,0" # set laser off
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    data_sent = "LASER:STAT? 1" # get state
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = int(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float == 0) :
        return True
    else:
        return False

def check_current_post_trigger(ser,grid):
    ser.read_all() # clear the serial buffers
    data_sent = "LASER:CURR? 1" # get laser current
    ser.write('{}\n'.format(data_sent).encode('utf-8')) # send it as utf-8
    data_received = ser.read_until(size=75) # read the reply (limited to 75 chars)
    data_float = 0
    try:
        data_received = data_received.decode('utf-8').rstrip()
        data_float = float(data_received)
    except:
        data_received = data_received.hex()
    grid.add_row("Data sent:", " ", data_sent)
    grid.add_row("Data received:", " ", data_received)
    if (data_float > 1) and (data_float < 20) :
        return True
    else:
        return False

def test_trigger(ser,verbose=False):
    console.print("Testing the laser trigger... ",end="")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    # a lot of things need to go right to pass this test!
    pass_result = "Fail"
    if check_current_pre_trigger(ser,grid):
        if check_power_set_mid_range(ser,grid):
            if check_laser_state_on(ser,grid):
                if check_current_while_triggered(ser,grid):
                    if check_laser_state_off(ser,grid):
                        if check_current_post_trigger(ser,grid):
                            pass_result = "Pass"
                            text = Text("[  OK  ]")
                            text.stylize("bold green", 0, 8)
                            console.print(text)
    if pass_result == "Fail":
        text = Text("[ FAIL ]")
        text.stylize("bold red", 0, 8)
        console.print(text)
    grid.add_row("Status:", " ", pass_result)
    if verbose == True:
        console.print(grid)
    if pass_result == "Fail":
        console.print(grid)
        return False
    return True

def test_power_ramp(ser,verbose=False):
    console.print("Testing the laser power setting... ")
    grid = Table.grid(expand=True)
    grid.add_column(justify="right", style="cyan")
    grid.add_column(style="yellow")
    grid.add_column(style="yellow")
    set_nums = np.linspace(0,254,32)
    get_nums = np.linspace(0,254,32)
    for i in set_nums:
        check_laser_state_on(ser,grid)
        check_power_set_mid_range(ser,grid,val=i)
        a,b = check_current_while_triggered(ser,grid)
        get_nums[int(i/8)] = b
    check_laser_state_off(ser,grid)
    print(asciiize(
        get_nums.tolist(),
        height=25,
        inter_points_margin=1,

        #x_ticks=list(range(1, 256,2)),
        y_ticks_decimal_places=0,

        x_axis_description='Setpoint (DAX)',
        y_axis_description='Current (mA)',

        title='Current setpoint',
        indentation=1
    ))
    #console.print(get_nums)

def meq_single_module_bringup_sequence(port):
    ser = serial.Serial(port=port,timeout=0.3)
    identity_check(ser)
    check_i2c_bus(ser)
    set_pac_addr(ser)
    set_serial(ser)
    set_wavelength(ser)
    set_pmax(ser)
    save_settings(ser)
    restart_device(ser)
    ser = serial.Serial(port=port,timeout=0.3)
    test_temperature(ser)
    test_voltage(ser)
    test_current(ser)
    test_trigger(ser)
    test_power_ramp(ser)

if __name__ == "__main__":
    console.rule("Starting bringup tests")
    port = serial_port_selector()
    console.print("Connecting to device on port {}".format(port))
    console.rule("Running device bringup test")
    meq_single_module_bringup_sequence(port)
    