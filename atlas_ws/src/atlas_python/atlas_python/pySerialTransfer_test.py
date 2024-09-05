from time import sleep
from pySerialTransfer import pySerialTransfer as txfer


SERIAL_DEVICE = '/dev/ttyACM0'
SENT_FLAG = False

class motorCmdStruct:
    status = 400
    cmd = 'b'
    arg1 = float(0)
    arg2 = float(0)
    arg3 = float(0)
    arg4 = float(0)


if __name__ == '__main__':
    try:
        testStruct = motorCmdStruct
        link = txfer.SerialTransfer(SERIAL_DEVICE)
        
        link.open()
        sleep(2) # allow some time for the Arduino to completely reset

        testStruct.arg1 = float(0)
        testStruct.arg2 = float(0)
        testStruct.status = 100
        
        while True:
            if not SENT_FLAG:
                print("sending")
                send_size = 0
                
                ###################################################################
                # Send a list
                ###################################################################
                send_size = link.tx_obj(testStruct.status, start_pos=send_size, val_type_override='i')
                send_size = link.tx_obj(testStruct.cmd, start_pos=send_size, val_type_override='c')
                send_size = link.tx_obj(testStruct.arg1, start_pos=send_size, val_type_override='f')
                send_size = link.tx_obj(testStruct.arg2, start_pos=send_size, val_type_override='f')
                send_size = link.tx_obj(testStruct.arg3, start_pos=send_size, val_type_override='f')
                send_size = link.tx_obj(testStruct.arg4, start_pos=send_size, val_type_override='f')
                
                ###################################################################
                # Transmit all the data to send in a single packet
                ###################################################################
                link.send(send_size)
                SENT_FLAG = True
            
            ###################################################################
            # Wait for a response and report any errors while receiving packets
            ###################################################################
            while not link.available():
                if link.status < 0:
                    if link.status == -1:
                        print('ERROR: CRC_ERROR')
                    elif link.status == -2:
                        print('ERROR: PAYLOAD_ERROR')
                    elif link.status == -3:
                        print('ERROR: STOP_BYTE_ERROR')
            
            print("reading")
            recSize = 0

            testStruct.status = link.rx_obj(obj_type='h', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['h']

            testStruct.cmd = link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            testStruct.arg1 = link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            testStruct.arg2 = link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            testStruct.arg3 = link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            testStruct.arg4 = link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
            link.close()

            ###################################################################
            # Display the received data
            ###################################################################
            print('RCVD: {} {} {} {} {} {}'.format(testStruct.status, testStruct.cmd, testStruct.arg1, testStruct.arg2, testStruct.arg3, testStruct.arg4))
            print(' ')
    
    except KeyboardInterrupt:
        try:
            link.close()
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            link.close()
        except:
            pass