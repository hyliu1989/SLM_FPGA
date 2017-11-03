# SLM control by FPGA

The mask file is loaded from a micro SD card which 1) is in FAT32 format and 2) has the mask file named "masks.npy" (all lower case and follow NumPy convention 1.0 or 2.0).

- [x] Test FIFO write: assuming that the image is a horizontal strip with vertical position controlled by SW
- [x] VGA display control syncronization 
	  (be noted that the two SYNC signals might change polarity (active high (current) -> active low)
	   if SLM uses regular VGA convention)
- [x] Test FIFO write: use a pattern with variations in each of its horizontal lines. (this checks if the horizontal sync is correct)
- [ ] Write random patterns into memory (64MB)
- [ ] Memory to FIFO with the id of line_to_load and the id of image_to_load
- [ ] MicroSD card reader and save images to memory (LED indicator for loading taking place)
- [ ] JTAG-UART communication for experimental commands
