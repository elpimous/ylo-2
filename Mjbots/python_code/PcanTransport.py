#!/usr/bin/python3 -B

# PcanTransport.py, a pcan ylo2 library
# automatic PCAN_PCIBUS selection, depending on controller ID

#-----------------------------------------------------------------

# Pcan M2 PEAK board has 4 PCAN_PCIBUS
# all 12 motors are linked 3 per 3 (each link_chain is one leg)
# power board, id = 32, is connected on the 3rd leg
# each leg is connected to one PCAN_PCIBUS(x) where x = 1 to 4

# each link_chain is terminated both side with 120 Ohn

##################################################################
#
#      just call lib as : 
#      ------------------
#
#      from PcanTransport import TRANSPORT as moteus_controller
#      d = moteus_controller(12)  # 12 => motor ID
#
##################################################################

import asyncio
import moteus

controllers_mapping = {1: 1, 2: 1, 3: 1, 4: 2, 5: 2, 6: 2, 7: 3, 8: 3, 9: 3, 10: 4, 11: 4, 12: 4, 32: 3}

def TRANSPORT(ID):
    channel = 'PCAN_PCIBUS'+str(controllers_mapping[ID])
    pcan_channel = moteus.PythonCan(interface='pcan', channel=channel)
    transport = moteus.Controller(id=ID, transport=pcan_channel)
    return transport