# SLM control by FPGA

- [x] Test FIFO write: assuming that the image is a horizontal strip with vertical position controlled by SW
- [x] VGA display control syncronization 
	  (be noted that the two SYNC signals might change polarity (active high (current) -> active low)
	   if SLM uses regular VGA convention)
- [x] Test FIFO write: use a pattern with variations in each of its horizontal lines. (this checks if the horizontal sync is correct)
- [x] Write vertical strip patterns into memory (64MB)
- [x] Memory to FIFO with the id of line_to_load and the id of image_to_load
- [x] JTAG-UART communication for downloading images to memory (7-segment indicators for loading taking place)
- [ ] JTAG-UART communication for experimental commands

# Usage
Compilation

1. Open Qsys inside Quartus Prime and generate codes.
1. Hit Generate HDL codes.
1. Start "Start Compilation (CTRL+L)"
