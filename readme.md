# SLM control by FPGA

- [x] Test FIFO write: assuming that the image is a horizontal strip with vertical position controlled by SW
- [x] VGA display control syncronization 
	  (be noted that the two SYNC signals might change polarity (active high (current) -> active low)
	   if SLM uses regular VGA convention)
- [x] Test FIFO write: use a pattern with variations in each of its horizontal lines. (this checks if the horizontal sync is correct)
- [x] Write vertical strip patterns into memory (64MB)
- [x] Memory to FIFO with the id of line_to_load and the id of image_to_load
- [x] JTAG-UART communication for downloading images to memory (7-segment indicators for loading taking place)
- [x] JTAG-UART communication for experimental commands
- [x] sequencer according the communicated information

# Usage
Compilation

1. Open Qsys inside Quartus Prime and generate codes.
1. Hit Generate HDL codes.
1. Start "Start Compilation (CTRL+L)"

Burning onto FPGA
1. Open "Programmer"
1. If it is a empty one, click Auto Detect and select 5CSEMA5.
1. Click Add File and choose output_files/SLMCtrl.sof
1. Click Hardware Setup and select DE1-SoC
1. Click Start

Burning onto the Flash drive next to FPGA so that FPGA is loaded at power-up everytime
0. (Chapter 8 of DE1-SoC user manual) (please search in google with "DE1 SoC user manual")

