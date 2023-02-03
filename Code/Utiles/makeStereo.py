import wave, array
import sys

def make_stereo(fileInp):
    ifile = wave.open(fileInp)
    print (ifile.getparams())
    # (1, 2, 44100, 2013900, 'NONE', 'not compressed')
    (nchannels, sampwidth, framerate, nframes, comptype, compname) = ifile.getparams()
    assert comptype == 'NONE'  # Compressed not supported yet
    array_type = {1:'B', 2: 'h', 4: 'l'}[sampwidth]
    left_channel = array.array(array_type, ifile.readframes(nframes))[::nchannels]
    ifile.close()

    stereo = 2 * left_channel
    stereo[0::2] = stereo[1::2] = left_channel

    fileOut = fileInp.replace('wav', 'stereo.wav')
    ofile = wave.open(fileOut, 'w')
    ofile.setparams((2, sampwidth, framerate, nframes, comptype, compname))
    ofile.writeframes(stereo)
    ofile.close()



if __name__ == "__main__":
    fileName = sys.argv[1]
    make_stereo(fileName)