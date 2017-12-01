/* Quartus Prime Version 16.0.0 Build 211 04/27/2016 SJ Lite Edition */
JedecChain;
	FileRevision(JESD32A);
	DefaultMfr(6E);

	P ActionCode(Ign)
		Device PartName(SOCVHPS) MfrSpec(OpMask(0));
	P ActionCode(Cfg)
		Device PartName(5CSEMA5F31) Path("T:/Herbert/Dropbox/MyLabStuffs/Research/phaseSLM2017/FPGA/SLM_FPGA/output_files/") File("SLMCtrl.sof") MfrSpec(OpMask(1));

ChainEnd;

AlteraBegin;
	ChainType(JTAG);
AlteraEnd;
