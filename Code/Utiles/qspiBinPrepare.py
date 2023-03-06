from ctypes import *
import re

MAX_FLASH_SIZE_KB = 0x1000000 / 0x400
SECTOR_SIZE = 0x10000
START_ADDRESS = 0x90000000
START_WAV_ADDRESS = START_ADDRESS + SECTOR_SIZE

# this value can't be changing without changes in MCU struct format
PADS_MAX = 3
MAX_SOUNDS = 0x10

class Notes:
    kick = 0x24
    
    snare_head = 0x26
    snare_rim = 0x28
    snare_x_stick = 0x25
    
    tom1_head = 0x30
    tom1_rim = 0x32
    
    tom2_head = 0x2D
    tom2_rim = 0x2F
    
    tom3_head = 0x2B
    tom3_rim = 0x3A
    
    tom4_head = 0x29
    tom4_rim = 0x27
    
    ride_bow = 0x33
    ride_bell = 0x35
    ride_edge = 0x3B
    
    crash_bow = 0x31
    crash_edge = 0x37
    
    crash2_bow = 0x39
    crash2_edge = 0x34
    
    hi_hat_open = 0x2E
    hi_hat_half_open = 0x17
    hi_hat_closed = 0x2A
    hi_hat_pedal_chick = 0x2C
    hi_hat_pedal_splash = 0x15
    

class MemmorySpace(Structure):
    _fields_ = [
        ('numberOfTotalPads', c_byte),
        ('soundsNumber', c_byte),
        ('padSavingsOffsets', c_uint64 * PADS_MAX),
        ('soundsAdresses', c_uint32 * MAX_SOUNDS)
    ]


class PadMemory(Structure):
    
    pads_counter = 0
    _list_ofWavFiles = []
    list_wav_addresses = [START_WAV_ADDRESS]
    bytes_array_wav_files = bytearray()
    
    _fields_ = (('sensitivity', c_uint8),        # 0
                ('threshold1', c_uint8),         # 1
                ('scantime', c_uint8),           # 2
                ('masktime', c_uint8),           # 3
                ('rimSensitivity', c_uint8),     # 4
                ('rimThreshold', c_uint8),       # 5
                ('curvetype', c_uint8),          # 6
                ('noteHead', c_uint8),           # 7
                ('noteRim', c_uint8),            # 8
                ('noteCup', c_uint8),            # 9
                ('soundHeadAddressId', c_uint8), # 10
                ('soundRimAddressId', c_uint8),  # 11
                ('soundCupAddressId', c_uint8),  # 12
                ('soundHeadVolumeDb', c_float),
                ('soundRimVolumeDb', c_float),
                ('soundCupVolumeDb', c_float))  

    def __init__(self,
                 sensitivity=100,
                 threshold1=99,
                 scantime=98,
                 masktime=97,
                 rimSensitivity=96,
                 rimThreshold=95,
                 curvetype=94,
                 noteHead=93,
                 noteRim=92,
                 noteCup=91,
                 soundHeadAddressId=0xFF,
                 soundRimAddressId = 0xFF,
                 soundCupAddressId = 0xFF,
                 soundHeadVolumeDb = -3.0,
                 soundRimVolumeDb = -3.0,
                 soundCupVolumeDb = -3.0): 
        PadMemory.pads_counter += 1
        assert PadMemory.pads_counter < 20, "Reached maximum of pads"
        super(PadMemory, self).__init__(sensitivity,
                                        threshold1,
                                        scantime,
                                        masktime,
                                        rimSensitivity,
                                        rimThreshold,
                                        curvetype,
                                        noteHead,
                                        noteRim,
                                        noteCup,
                                        soundHeadAddressId,
                                        soundRimAddressId,
                                        soundCupAddressId,
                                        soundHeadVolumeDb,
                                        soundRimVolumeDb,
                                        soundCupVolumeDb)
        
    def setHeadWavFile(self, filename:str):
        self.soundHeadAddressId = addWavFileToBinImage(filename)
        
    def setRimWavFile(self, filename:str):
        self.soundRimAddressId = addWavFileToBinImage(filename)
        
    def setCupWavFile(self, filename:str):
        self.soundCupAddressId = addWavFileToBinImage(filename)
          

class QspiGenerate:
    _num_of_pads = 0x6
    _num_of_sounds = 0x8
    def __init__(self) -> None:
        self.mem_sys_sector = MemmorySpace()
        
    def generatePads(self) -> bytearray:
        # All wav files must be stereo in 48 kHz.
        kick = PadMemory(noteHead=Notes.kick)
        kick.setHeadWavFile("kick.wav")
        snare = PadMemory(noteHead=Notes.snare_head, noteRim=Notes.snare_rim)
        snare.setHeadWavFile("snare.wav")
        tom1 = PadMemory(noteHead=Notes.tom1_head, noteRim=Notes.tom1_rim)
        tom1.setHeadWavFile("tomF.wav")
        ride = PadMemory(noteHead=Notes.ride_bow, noteRim=Notes.ride_edge, noteCup=Notes.ride_bell)
        ride.setHeadWavFile("crash.wav")
        return bytes(kick) + bytes(snare) + bytes(tom1) + bytes(ride)
                

    def create_mem_sys_sector(self) -> bytearray:
        pads_array = self.generatePads()
        
        self.mem_sys_sector.numberOfTotalPads = PadMemory.pads_counter
        self.mem_sys_sector.soundsNumber = self._num_of_sounds
        self.fill_pads_offsets()
        
        adresses = PadMemory.list_wav_addresses
        self.fill_sounds_address(adresses)
        
        tempToOut = bytearray(self.mem_sys_sector) + pads_array
        tempToOut = self.fill_FF_to_the_end_sector(tempToOut)
        
        tempToOut += PadMemory.bytes_array_wav_files
    
        bin_size = len(tempToOut)/0x400
        assert (bin_size + 16) < MAX_FLASH_SIZE_KB, "Bin size is very big, please decrease wav volume"
        print("bin size is " + str(bin_size)+ " KB")
        printByes(tempToOut[:0x100])
        return tempToOut
            
    def fill_pads_offsets(self):
        for i in range(PADS_MAX):
            self.mem_sys_sector.padSavingsOffsets[i] = 0xFFFFFFFFFFFFFFFF
            
    def fill_sounds_address(self, adresses: dict):
        num_adresses = len(adresses)
        assert num_adresses < MAX_SOUNDS, "Number of adresses must be less than MAX_SOUNDS"
        for i in range(num_adresses):
            self.mem_sys_sector.soundsAdresses[i] = adresses[i]
        for i in range(num_adresses, MAX_SOUNDS):
            self.mem_sys_sector.soundsAdresses[i] = 0xFFFFFFFF
            
    def fill_FF_to_the_end_sector(self, input: bytearray) -> bytearray:
        input += bytes([0xFF])* (SECTOR_SIZE - len(input))
        return input

def writeByteArrayToFile(bin: bytearray, filename = "qspi.bin"):
    file_to_write = open(filename, 'wb')
    file_to_write.write(bin)
    file_to_write.close()
    print("Bin image was writed success in file " + filename)

def printByes(buffer: bytearray):
    buffer += bytes(0x10) # this show last string if using re.compile("(.{80})")
    bytesInHex = ' '.join('0x{:02x}'.format(x) for x in bytearray(buffer))
    
    p = re.compile("(.{80})")
    for i in p .finditer(bytesInHex):
        print(format(int(i.start()/5), '#06x'), i.group())


def addWavFileToBinImage(filename:str) -> c_uint8:
    PadMemory._list_ofWavFiles.append(filename)
    parameter = len(PadMemory._list_ofWavFiles) - 1
    with open(filename, 'rb') as file_to_read:
        content = file_to_read.read()
        
    PadMemory.bytes_array_wav_files += content
    PadMemory.list_wav_addresses.append(len(PadMemory.bytes_array_wav_files) + START_WAV_ADDRESS)
    return parameter  

gen = QspiGenerate()
binImage = gen.create_mem_sys_sector()
writeByteArrayToFile(binImage)


# print("wav addresses is")
# for i in range(len(PadMemory.list_wav_addresses)):
#     print(str(hex(PadMemory.list_wav_addresses[i])))
