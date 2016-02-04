import sys
import os
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from datetime import datetime
import time
from EPD import EPD

WHITE = 1
BLACK = 0

# fonts are in different places on Raspbian/Angstrom so search
possible_fonts = [
    '/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf',            # Debian B.B
    '/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf',   # Debian B.B
    '/usr/share/fonts/truetype/ttf-dejavu/DejaVuSansMono-Bold.ttf',   # R.Pi
    '/usr/share/fonts/truetype/freefont/FreeMono.ttf',                # R.Pi
    '/usr/share/fonts/truetype/LiberationMono-Bold.ttf',              # B.B
    '/usr/share/fonts/truetype/DejaVuSansMono-Bold.ttf',              # B.B
    '/usr/share/fonts/TTF/FreeMonoBold.ttf',                          # Arch
    '/usr/share/fonts/TTF/DejaVuSans-Bold.ttf'                        # Arch
]


FONT_FILE = ''
for f in possible_fonts:
    if os.path.exists(f):
        FONT_FILE = f
        break

if '' == FONT_FILE:
    raise 'no font file found'

FONT_SIZE  = 27

MAX_START = 0xffff



def main():
	print 'EPaper Display Program'
	print 'Reads from /home/pi/epaper/message.txt'
	print

	epd = EPD()

	print('panel = {p:s} {w:d} x {h:d}  version={v:s}'.format(p=epd.panel, w=epd.width, h=epd.height, v=epd.version))

	#epd.clear()

	read_file_and_display('/home/pi/epaper/message.txt', epd)


def read_file_and_display(filename, epd):

	# initially set all white background
	image = Image.new('1', epd.size, WHITE)

	# prepare for drawing
	draw = ImageDraw.Draw(image)
	width, height = image.size

	display_font = ImageFont.truetype(FONT_FILE, FONT_SIZE)

	# clear the display buffer
	draw.rectangle((0, 0, width, height), fill=WHITE, outline=WHITE)

	draw.rectangle((2, 2, width - 2, height - 2), fill=WHITE, outline=BLACK)
            
	rowPos = 10
	rowIncrement = 30

	with open(filename) as f:
		for line in f:
			if line.endswith('\n'):
				line = line[:-1]
			if line.endswith('\r'):
				line = line[:-1]

			draw.text((5, rowPos), line, fill=BLACK, font=display_font)

			rowPos = rowPos + rowIncrement

			print(line)

	epd.display(image)
        
	epd.update()

if "__main__" == __name__:
        main()
        sys.exit('Done!')
