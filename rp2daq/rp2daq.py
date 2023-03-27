#!/usr/bin/python3  
#-*- coding: utf-8 -*-
"""
rp2daq.py  (c) Filip Dominec 2020-2022, MIT licensed

This module uses c_code_parser.py to auto-generate the binary message interface. 
Then it connects to Raspberry Pi Pico to control various hardware. 

The methods provided here aim to make the hardware control as convenient as possible.  

More information and examples on https://github.com/FilipDominec/rp2daq or in README.md
"""


MIN_FW_VER = 210400

import atexit
from collections import deque
import logging
import os
import queue
import serial
from serial.tools import list_ports 
import struct
import sys
import threading
import time
import traceback
import tkinter
import types
import pathlib
import re
import os

def remove_c_comments(f):
    f = re.sub("\/\*.+?\*\/", "", f, flags=re.M+re.S) # rm block comments
    f = re.sub("\/\/.*", "", f) # rm line comments
    return f


def get_next_code_block(s, lbrace="{", rbrace="}"):
    """
    Returns the content of a first brace-delimited block, properly handling nested 
    sub-blocks. 
    Todo: Braces in comments, nor preprocessor tricks are NOT handled.
    >>> get_next_code_block("aaaaaaa{bbb{ccc{dd}cc}b{ccccc}b}a{bb}a")
    bbb{ccc{dd}cc}b{ccccc}b
    """
    s = s.split(lbrace,1)[1]
    #print(s)

    found_block, nest_level = "", 1
    for new_chunk in s.split(rbrace):
        nest_level += new_chunk.count(lbrace) - 1 
        found_block += new_chunk 
        if nest_level == 0: 
            return found_block
        found_block += rbrace

def get_prev_code_block(s, lbrace="{", rbrace="}"):
    return get_next_code_block(s[::-1], lbrace=rbrace, rbrace=lbrace)[::-1]

def generate_command_codes(C_code):
    command_table_match = re.search(r"message_descriptor message_table", C_code, 
            flags=re.M+re.S)
    command_table_code = get_next_code_block(C_code[command_table_match.span()[1]:])
    command_table_code = remove_c_comments(command_table_code)
    return {c.group():n//2 for n,c in enumerate(re.finditer(r"\w+", command_table_code, 
        flags=re.M+re.S)) if not c.group().endswith("_report")}

def generate_command_binary_interface():
    """ Parses the RP2DAQ firmware in C language, searching for the table of commands, and 
    then the binary structures each command is supposed to accept.
    Returns a dict of functions which transmit such binary messages, exactly matching the 
    message specification in the C code. For convenience, these functions have properly named 
    parameters, possibly with default values, and with checks for their minimum/maximum 
    allowed values. """
    # Fixme: error-prone assumption that args are always the 1st block

    C_code = gather_C_code()
    command_codes = generate_command_codes(C_code)

    func_dict = {}
    markdown_docs = ""
    for command_name in command_codes.keys():
        command_code = command_codes[command_name]
        q = re.search(f"void\\s+{command_name}\\s*\\(\\)", C_code)
        func_body = get_next_code_block(C_code[q.span()[1]:]) # code enclosed by closest brace block
        args_struct = get_next_code_block(func_body) 

        struct_signature, cmd_length = "", 0
        arg_names, arg_defaults = [], []

        exec_header = f""
        exec_prepro = f""
        exec_struct = f"" 
        exec_stargs = f""

        try:
            raw_docstring = get_next_code_block(func_body, lbrace="/*", rbrace="*/").strip()
            raw_docstring = re.sub(r"\n(\s*\*)? ?", "\n", raw_docstring)  # rm leading asterisks, keep indent
        except IndexError: 
            raw_docstring = ""
        exec_docstring = f"{raw_docstring.replace('__','')}\n\n"

        #print(exec_docstring)


        param_docstring = ""
        for line in re.finditer(r"(u?)int(8|16|32)_t\s+([\w,]*)(.*)",  args_struct):
            unsigned, bits, arg_name_multi, line_comments = line.groups()
            bit_width_code = {8:'b', 16:'h', 32:'i'}[int(bits)]

            arg_attribs = {}
            arg_comment = ""
            for commentoid in re.split("\\s+", line_comments):
                u = re.match("(\\w*)=(-?\\d*)", commentoid)
                if u:
                    arg_attribs[u.groups()[0]] = u.groups()[1]
                elif not commentoid in ("", "//", ";"):
                    arg_comment += " " + commentoid
                comment = arg_comment.strip()

            for arg_name in arg_name_multi.split(","):
                cmd_length += int(bits)//8
                exec_struct += bit_width_code.upper() if unsigned else bit_width_code
                exec_stargs += f"\n\t\t\t{arg_name},"
                arg_names.append(arg_name)

                d = arg_attribs.get("default")
                if d:
                    exec_header += f"{arg_name}={d}, "
                else: 
                    exec_header += f"{arg_name}, "

                m = arg_attribs.get("min")
                if m is not None:
                    exec_prepro += f"\tassert {arg_name} >= {arg_attribs['min']}, "+\
                            f"'Minimum value for {arg_name} is {arg_attribs['min']}'\n"

                m = arg_attribs.get("max")
                if m is not None:
                    exec_prepro += f"\tassert {arg_name} <= {arg_attribs['max']},"+\
                            f"'Maximum value for {arg_name} is {arg_attribs['max']}'\n"

                param_docstring += f"  * {arg_name} {':' if comment else ''} {comment} \n" #  TODO print range  (min=0 max= 2³²-1)

        param_docstring += f"  * _callback : optional report handling function; if set, this command becomes asynchronous (does not wait for report) \n\n"

        #exec_docstring += "Returns:\n\n" # TODO analyze report structure (and comments therein)

        # Append extracted docstring to the overall API reference
        markdown_docs += f"\n\n## {command_name}\n\n{raw_docstring}\n\n"
        markdown_docs += f"__Call signature:__\n\n`{command_name}({exec_header} _callback=None)`\n\n"
        markdown_docs += f"__Parameters__:\n\n{param_docstring}\n"
        #markdown_docs += f"{raw_docstring}\n\n#### Arguments:"

        # TODO once 16-bit msglen enabled: cmd_length will go +3, and 1st struct Byte must change to Half-int 
        exec_msghdr = f"', {cmd_length+2}, {command_code}, "
        code = f"def {command_name}(self,{exec_header} _callback=None):\n" +\
                f'\t"""{raw_docstring}\n\nParameters:\n{param_docstring}"""\n' +\
                exec_prepro +\
                f"\tif {command_code} not in self.sync_report_cb_queues.keys():\n" +\
                f"\t\tself.sync_report_cb_queues[{command_code}] = queue.Queue()\n" +\
                f"\tself.report_callbacks[{command_code}] = _callback\n" +\
                f"\tself.port.write(struct.pack('<BB{exec_struct}{exec_msghdr}{exec_stargs}))\n" +\
                f"\tif not _callback:\n" +\
                f"\t\treturn self.default_blocking_callback({command_code})"



        func_dict[command_name] = code  # returns Python code
    return func_dict, markdown_docs

def generate_report_binary_interface():
    C_code = gather_C_code()
    command_codes = generate_command_codes(C_code)
    report_lengths, report_header_signatures, arg_names_for_reports = {}, {}, {}
    for report_name, report_number in command_codes.items():
        q = re.search(f"}}\\s*{report_name}_report", C_code)
        #print(report_number, report_name,q)
        report_struct_code = get_prev_code_block(C_code[:q.span()[0]+1]) # code enclosed by closest brace block
        # FIXME THIS IS JUST USELESS ERROR:  AttributeError: 'NoneType' object has no attribute 'span'
        #print(f"{report_name=} {report_struct_code=}") 

        report_header_signature, report_length = "<", 0
        arg_names, arg_defaults = [], []

        for line in re.finditer(r"(u?)int(8|16|32)_t\s+([\w,]*)([; \t\w=\/]*)",  report_struct_code):
            unsigned, bits, arg_name_multi, line_comments = line.groups()
            bit_width_code = {8:'b', 16:'h', 32:'i'}[int(bits)]

            for arg_name in arg_name_multi.split(","):
                report_length += int(bits)//8
                report_header_signature += bit_width_code.upper() if unsigned else bit_width_code
                arg_names.append(arg_name)
        report_lengths[report_number] = report_length
        assert report_length > 0, "every report has to contain at least 1 byte, troubles ahead"
        report_header_signatures[report_number] = report_header_signature
        arg_names_for_reports[report_number] = arg_names

    return report_lengths, report_header_signatures, arg_names_for_reports

def gather_C_code():
    C_code = open(os.path.dirname(os.path.realpath(__file__))+'/rp2daq.c').read()
    C_code += open(os.path.dirname(os.path.realpath(__file__))+'/adc_internal.c').read()
    C_code += open(os.path.dirname(os.path.realpath(__file__))+'/identify.c').read()
    C_code += open(os.path.dirname(os.path.realpath(__file__))+'/pin_out.c').read()
    C_code += open(os.path.dirname(os.path.realpath(__file__))+'/pwm.c').read()
    C_code += open(os.path.dirname(os.path.realpath(__file__))+'/stepper.c').read()

    return C_code


def init_error_msgbox():  # error handling with a graphical message box
    def myerr(exc_type, exc_value, tb): 
        message = '\r'.join(traceback.format_exception(exc_type, exc_value, tb))
        logging.error(message)
        from tkinter import messagebox
        messagebox.showerror(title=exc_value, message=message)
    sys.excepthook = myerr



class Rp2daq():
    def __init__(self, required_device_id="", verbose=False):

        logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO, 
                format='%(asctime)s (%(threadName)-9s) %(message)s',) # filename='rp2.log',

        # Most of the technicalities are delegated to the following class. The Rp2daq's namespace 
        # will be dynamically populated with useful commands
        self._i = Rp2daq_internals(externals=self, required_device_id=required_device_id, verbose=verbose)

        atexit.register(self.quit)


    def quit(self):
        """Clean termination of tx/rx threads, and explicit releasing of serial ports (for win32) """ 
        # TODO: add device reset option
        self._i.run_event.clear()
        self._i.port.close()



class Rp2daq_internals(threading.Thread):
    def __init__(self, externals, required_device_id="", verbose=False):

        self._e = externals

        self._register_commands()

        time.sleep(.05)
        self.port = self._find_device(required_device_id, required_firmware_version=MIN_FW_VER)

        ## Asynchronous communication using threads
        self.sleep_tune = 0.001

        threading.Thread.__init__(self) 
        self.data_receiving_thread = threading.Thread(target=self._data_receiver, daemon=True)
        self.report_processing_thread = threading.Thread(target=self._report_processor, daemon=True)
        self.callback_dispatching_thread = threading.Thread(target=self._callback_dispatcher, daemon=True)

        self.rx_bytes = deque()
        self.run_event = threading.Event()
        self.data_receiving_thread.start()
        self.report_processing_thread.start()
        self.callback_dispatching_thread.start()
        self.run_event.set()


    def _register_commands(self):
        # TODO 0: search C code for version, check it matches that one returned by Raspberry Pi at runtime
        # #define FIRMWARE_VERSION {"rp2daq_220720_"}
        # self.expected_firmware_v = 

        names_codes, markdown_docs  = generate_command_binary_interface()
        for cmd_name, cmd_code in names_codes.items():
            exec(cmd_code)
            setattr(self._e, cmd_name, types.MethodType(locals()[cmd_name], self))

        # Search C code for report structs & generate automatically:
        self.sync_report_cb_queues = {}
        self.async_report_cb_queue = queue.Queue()
        self.report_header_lenghts, self.report_header_formats, self.report_header_varnames = \
                generate_report_binary_interface()

        # Register callbacks (to dispatch reports as they arrive)
        self.report_callbacks = {} 


    def _data_receiver(self):
        self.run_event.wait()

        while self.run_event.is_set():
                inwaiting = self.port.inWaiting()
                if inwaiting:
                    c = self.port.read(inwaiting)
                    self.rx_bytes.extend(c)
                else:
                    time.sleep(self.sleep_tune)

    def _report_processor(self):
        """
        Thread to continuously check for incoming data.
        When a byte comes in, place it onto the deque.
        """
        def rx_at_least_bytes(length):
            while len(self.rx_bytes) < length:
                #c = self.port.read(w)
                #self.rx_bytes.extend(c) # superfluous bytes are kept in deque for later use
                time.sleep(self.sleep_tune)
            return [self.rx_bytes.popleft() for _ in range(length)]

        self.run_event.wait()

        while self.run_event.is_set():
            try:
                if len(self.rx_bytes):
                    # report_header_bytes will be populated with the received data for the report
                    report_type = self.rx_bytes.popleft()
                    packet_length = self.report_header_lenghts[report_type] - 1

                    report_header_bytes = rx_at_least_bytes(packet_length)

                    #logging.debug(f"received packet header {report_type=} {report_header_bytes=} {bytes(report_header_bytes)=}")

                    report_args = struct.unpack(self.report_header_formats[report_type], 
                            bytes([report_type]+report_header_bytes))
                    cb_kwargs = dict(zip(self.report_header_varnames[report_type], report_args))

                    data_bytes = []
                    dc, dbw = cb_kwargs.get("_data_count",0), cb_kwargs.get("_data_bitwidth",0)
                    if dc and dbw:
                        payload_length = -((-dc*dbw)//8)  # integer division is like floor(); this makes it ceil()
                        #print("  PL", payload_length)

                        data_bytes = rx_at_least_bytes(payload_length)

                        if dbw == 8:
                            cb_kwargs["data"] = data_bytes
                        elif dbw == 12:      # decompress 3B  into pairs of 12b values & flatten
                            odd = [a + ((b&0xF0)<<4)  for a,b
                                    in zip(data_bytes[::3], data_bytes[1::3])]
                            even = [(c&0xF0)//16+(b&0x0F)*16+(c&0x0F)*256  for b,c
                                    in zip(                   data_bytes[1:-1:3], data_bytes[2::3])]
                            cb_kwargs["data"] = [x for l in zip(odd,even) for x in l]
                            if len(odd)>len(even): cb_kwargs["data"].append(odd[-1])

                        elif dbw == 16:      # using little endian byte order everywhere
                            cb_kwargs["data"] = [a+(b<<8) for a,b in zip(data_bytes[:-1:2], data_bytes[1::2])]

                        #if max(cb_kwargs["data"]) > 2250: print(data_bytes) # XXX
                    

                    cb = self.report_callbacks.get(report_type, False) # false for unexpected reports
                    if cb:
                        #logging.debug("CALLING CB {cb_kwargs}")
                        #print(f"CALLING CB {cb} {cb_kwargs}")
                        ## TODO: enqueue to be called by yet another thread (so that sync cmds work within callbacks,too)
                        ## TODO: check if sync cmd works correctly after async cmd (of the same type)
                        #cb(**cb_kwargs)
                        self.async_report_cb_queue.put((cb, cb_kwargs))
                    elif cb is None: # expected report from blocking command
                        #print(f"UNBLOCKING CB {report_type=} {cb_kwargs}")
                        self.sync_report_cb_queues[report_type].put(cb_kwargs) # unblock default callback (& send it data)
                    elif cb is False: # unexpected report, from command that was not yet called in this script instance
                        #print(f"Warning: Got callback that was not asked for\n\tDebug info: {cb_kwargs}")
                        pass 
                else:
                    time.sleep(self.sleep_tune)

            except OSError:
                logging.error("Device disconnected")
                self._e.quit()

    def _callback_dispatcher(self):
        """
        """
        self.run_event.wait()

        while self.run_event.is_set():
            (cb, cb_kwargs) = self.async_report_cb_queue.get()
            cb(**cb_kwargs)

    def default_blocking_callback(self, command_code): # 
        """
        Any command called without explicit `_callback` argument is blocking - i.e. the thread
        that called the command waits here until a corresponding report arrives. This is good 
        practice only if quick response from device is expected, or your script uses 
        multithreading. Otherwise your program flow would be stalled for a while here.

        This function is called from *autogenerated* code for each command, iff no _callback
        is specified.
        """
        #print("DBC WAITING", command_code)
        kwargs = self.sync_report_cb_queues[command_code].get() # waits until default callback unblocked
        return kwargs

    def _find_device(self, required_device_id, required_firmware_version=0):
        """
        Seeks for a compatible rp2daq device on USB, checking for its firmware version and, if 
        specified, for its particular unique vendor name.
        """

        VID, PID = 0x2e8a, 0x000a 
        port_list = list_ports.comports()

        for port_name in port_list:
            # filter out ports, without disturbing previously connected devices 
            if not port_name.hwid.startswith("USB VID:PID=2E8A:000A SER="+required_device_id.upper()):
                continue
            #print(f"port_name.hwid={port_name.hwid}")
            try_port = serial.Serial(port=port_name.device, timeout=0.01)

            try:
                #try_port.reset_input_buffer(); try_port.flush()
                #time.sleep(.05) # 50ms round-trip time is enough

                # the "identify" command is hard-coded here, as the receiving threads are not ready yet
                try_port.write(struct.pack(r'<BBB', 1, 0, 1)) 
                time.sleep(.15) # 50ms round-trip time is enough
                bytesToRead = try_port.inWaiting() 
                assert bytesToRead == 1+2+1+30
                id_data = try_port.read(bytesToRead)[4:] 
            except:
                id_data = b''

            if not id_data[:6] == b'rp2daq': 
                logging.info(f"\tport open, but device does not identify itself as rp2daq: {id_data}" )
                continue

            version_signature = id_data[7:13]
            if not version_signature.isdigit() or int(version_signature) < required_firmware_version:
                logging.warning(f"rp2daq device firmware has version {version_signature.decode('utf-8')},\n" +\
                        f"older than this script's {MIN_FW_VER}.\nPlease upgrade firmware " +\
                        "or override this error using 'required_firmware_version=0'.")
                continue

            if isinstance(required_device_id, str): # optional conversion
                required_device_id = required_device_id.replace(":", "")
            found_device_id = id_data[14:]
            if required_device_id and found_device_id != required_device_id:
                logging.info(f"found an rp2daq device, but its ID {found_device_id} does not match " + 
                        f"required {required_device_id}")
                continue

            logging.info(f"connected to rp2daq device with manufacturer ID = {found_device_id.decode()}")
            return try_port
        else:
            msg = "Error: could not find any matching rp2daq device"
            logging.critical(msg)
            raise RuntimeError(msg)



if __name__ == "__main__":
    print("Note: Running this module as a standalone script will only try to connect to a RP2 device.")
    print("\tSee the 'examples' directory for further uses.")
    rp = Rp2daq()       # tip you can use e.g. required_device_id='01020304050607'
    t0 = time.time()

    #rp.pin_set(11, 1, high_z=1, pull_up=1)
    #print(rp.pin_get(11))
    #rp.pwm_configure_pair(0, clkdiv=255, wrap_value=500)
    #rp.pwm_set_value(0, 200)
    #rp.pwm_configure_pair(0, clkdiv=255, wrap_value=500)
    #rp.pwm_set_value(1, 100)
    #rp.pwm_configure_pair(2, clkdiv=255, wrap_value=1000)
    #rp.pwm_set_value(2, 300)



