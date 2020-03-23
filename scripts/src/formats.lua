-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   formats.lua
--
--   Rules for building formats
--
---------------------------------------------------------------------------
function formatsProject(_target, _subtarget)
project "formats"
	uuid (os.uuid("formats-" .. _target .."_" .. _subtarget))
	kind (LIBTYPE)
	targetsubdir(_target .."_" .. _subtarget)
	addprojectflags()

	options {
		"ArchiveSplit",
	}

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		ext_includedir("zlib"),
	}

	files {
		MAME_DIR .. "src/lib/formats/ioprocs.cpp",
		MAME_DIR .. "src/lib/formats/ioprocs.h",
		MAME_DIR .. "src/lib/formats/imageutl.cpp",
		MAME_DIR .. "src/lib/formats/imageutl.h",

		MAME_DIR .. "src/lib/formats/cassimg.cpp",
		MAME_DIR .. "src/lib/formats/cassimg.h",
		MAME_DIR .. "src/lib/formats/wavfile.cpp",
		MAME_DIR .. "src/lib/formats/wavfile.h",

		MAME_DIR .. "src/lib/formats/flopimg.cpp",
		MAME_DIR .. "src/lib/formats/flopimg.h",

		MAME_DIR .. "src/lib/formats/cqm_dsk.cpp",
		MAME_DIR .. "src/lib/formats/cqm_dsk.h",
		MAME_DIR .. "src/lib/formats/dsk_dsk.cpp",
		MAME_DIR .. "src/lib/formats/dsk_dsk.h",
		MAME_DIR .. "src/lib/formats/td0_dsk.cpp",
		MAME_DIR .. "src/lib/formats/td0_dsk.h",
		MAME_DIR .. "src/lib/formats/hxchfe_dsk.cpp",
		MAME_DIR .. "src/lib/formats/hxchfe_dsk.h",
		MAME_DIR .. "src/lib/formats/hxcmfm_dsk.cpp",
		MAME_DIR .. "src/lib/formats/hxcmfm_dsk.h",
		MAME_DIR .. "src/lib/formats/mfi_dsk.cpp",
		MAME_DIR .. "src/lib/formats/mfi_dsk.h",
		MAME_DIR .. "src/lib/formats/imd_dsk.cpp",
		MAME_DIR .. "src/lib/formats/imd_dsk.h",
		MAME_DIR .. "src/lib/formats/ipf_dsk.cpp",
		MAME_DIR .. "src/lib/formats/ipf_dsk.h",
		MAME_DIR .. "src/lib/formats/d88_dsk.cpp",
		MAME_DIR .. "src/lib/formats/d88_dsk.h",
		MAME_DIR .. "src/lib/formats/dfi_dsk.cpp",
		MAME_DIR .. "src/lib/formats/dfi_dsk.h",
		MAME_DIR .. "src/lib/formats/fdi_dsk.cpp",
	}

--------------------------------------------------
--
--@src/lib/formats/2d_dsk.h,FORMATS["2D_DSK"] = true
--------------------------------------------------

if (FORMATS["2D_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/2d_dsk.cpp",
		MAME_DIR.. "src/lib/formats/2d_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/a26_cas.h,FORMATS["A26_CAS"] = true
--------------------------------------------------

if (FORMATS["A26_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/a26_cas.cpp",
		MAME_DIR.. "src/lib/formats/a26_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/a5105_dsk.h,FORMATS["A5105_DSK"] = true
--------------------------------------------------

if (FORMATS["A5105_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/a5105_dsk.cpp",
		MAME_DIR.. "src/lib/formats/a5105_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/abc800_dsk.h,FORMATS["ABC800_DSK"] = true
--------------------------------------------------

if (FORMATS["ABC800_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/abc800_dsk.cpp",
		MAME_DIR.. "src/lib/formats/abc800_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/abcfd2_dsk.h,FORMATS["ABCFD2_DSK"] = true
--------------------------------------------------

if (FORMATS["ABCFD2_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/abcfd2_dsk.cpp",
		MAME_DIR.. "src/lib/formats/abcfd2_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ace_tap.h,FORMATS["ACE_TAP"] = true
--------------------------------------------------

if (FORMATS["ACE_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ace_tap.cpp",
		MAME_DIR.. "src/lib/formats/ace_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/acorn_dsk.h,FORMATS["ACORN_DSK"] = true
--------------------------------------------------

if (FORMATS["ACORN_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/acorn_dsk.cpp",
		MAME_DIR.. "src/lib/formats/acorn_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/adam_cas.h,FORMATS["ADAM_CAS"] = true
--------------------------------------------------

if (FORMATS["ADAM_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/adam_cas.cpp",
		MAME_DIR.. "src/lib/formats/adam_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/adam_dsk.h,FORMATS["ADAM_DSK"] = true
--------------------------------------------------

if (FORMATS["ADAM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/adam_dsk.cpp",
		MAME_DIR.. "src/lib/formats/adam_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/afs_dsk.h,FORMATS["AFS_DSK"] = true
--------------------------------------------------

if (FORMATS["AFS_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/afs_dsk.cpp",
		MAME_DIR.. "src/lib/formats/afs_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ami_dsk.h,FORMATS["AMI_DSK"] = true
--------------------------------------------------

if (FORMATS["AMI_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ami_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ami_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ap2_dsk.h,FORMATS["AP2_DSK"] = true
--------------------------------------------------

if (FORMATS["AP2_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ap2_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ap2_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/apd_dsk.h,FORMATS["APD_DSK"] = true
--------------------------------------------------

if (FORMATS["APD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/apd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/apd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/apf_apt.h,FORMATS["APF_APT"] = true
--------------------------------------------------

if (FORMATS["APF_APT"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/apf_apt.cpp",
		MAME_DIR.. "src/lib/formats/apf_apt.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/apollo_dsk.h,FORMATS["APOLLO_DSK"] = true
--------------------------------------------------

if (FORMATS["APOLLO_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/apollo_dsk.cpp",
		MAME_DIR.. "src/lib/formats/apollo_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/applix_dsk.h,FORMATS["APPLIX_DSK"] = true
--------------------------------------------------

if (FORMATS["APPLIX_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/applix_dsk.cpp",
		MAME_DIR.. "src/lib/formats/applix_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/apridisk.h,FORMATS["APRIDISK"] = true
--------------------------------------------------

if (FORMATS["APRIDISK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/apridisk.cpp",
		MAME_DIR.. "src/lib/formats/apridisk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ap_dsk35.h,FORMATS["AP_DSK35"] = true
--------------------------------------------------

if (FORMATS["AP_DSK35"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ap_dsk35.cpp",
		MAME_DIR.. "src/lib/formats/ap_dsk35.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/asst128_dsk.h,FORMATS["ASST128_DSK"] = true
--------------------------------------------------

if (FORMATS["ASST128_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/asst128_dsk.cpp",
		MAME_DIR.. "src/lib/formats/asst128_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/atari_dsk.h,FORMATS["ATARI_DSK"] = true
--------------------------------------------------

if (FORMATS["ATARI_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/atari_dsk.cpp",
		MAME_DIR.. "src/lib/formats/atari_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/atom_dsk.h,FORMATS["ATOM_DSK"] = true
--------------------------------------------------

if (FORMATS["ATOM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/atom_dsk.cpp",
		MAME_DIR.. "src/lib/formats/atom_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/atom_tap.h,FORMATS["ATOM_TAP"] = true
--------------------------------------------------

if (FORMATS["ATOM_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/atom_tap.cpp",
		MAME_DIR.. "src/lib/formats/atom_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/basicdsk.h,FORMATS["BASICDSK"] = true
--------------------------------------------------

if (FORMATS["BASICDSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/basicdsk.cpp",
		MAME_DIR.. "src/lib/formats/basicdsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/bw12_dsk.h,FORMATS["BW12_DSK"] = true
--------------------------------------------------

if (FORMATS["BW12_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/bw12_dsk.cpp",
		MAME_DIR.. "src/lib/formats/bw12_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/bw2_dsk.h,FORMATS["BW2_DSK"] = true
--------------------------------------------------

if (FORMATS["BW2_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/bw2_dsk.cpp",
		MAME_DIR.. "src/lib/formats/bw2_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/c3040_dsk.h,FORMATS["C3040_DSK"] = true
--------------------------------------------------

if (FORMATS["C3040_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/c3040_dsk.cpp",
		MAME_DIR.. "src/lib/formats/c3040_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/c4040_dsk.h,FORMATS["C4040_DSK"] = true
--------------------------------------------------

if (FORMATS["C4040_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/c4040_dsk.cpp",
		MAME_DIR.. "src/lib/formats/c4040_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/c8280_dsk.h,FORMATS["C8280_DSK"] = true
--------------------------------------------------

if (FORMATS["C8280_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/c8280_dsk.cpp",
		MAME_DIR.. "src/lib/formats/c8280_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/camplynx_cas.h,FORMATS["CAMPLYNX_CAS"] = true
--------------------------------------------------

if (FORMATS["CAMPLYNX_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/camplynx_cas.cpp",
		MAME_DIR.. "src/lib/formats/camplynx_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/camplynx_dsk.h,FORMATS["CAMPLYNX_DSK"] = true
--------------------------------------------------

if (FORMATS["CAMPLYNX_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/camplynx_dsk.cpp",
		MAME_DIR.. "src/lib/formats/camplynx_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cbm_crt.h,FORMATS["CBM_CRT"] = true
--------------------------------------------------

if (FORMATS["CBM_CRT"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cbm_crt.cpp",
		MAME_DIR.. "src/lib/formats/cbm_crt.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cbm_tap.h,FORMATS["CBM_TAP"] = true
--------------------------------------------------

if (FORMATS["CBM_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cbm_tap.cpp",
		MAME_DIR.. "src/lib/formats/cbm_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ccvf_dsk.h,FORMATS["CCVF_DSK"] = true
--------------------------------------------------

if (FORMATS["CCVF_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ccvf_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ccvf_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cd90_640_dsk.h,FORMATS["CD90_640_DSK"] = true
--------------------------------------------------

if (FORMATS["CD90_640_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cd90_640_dsk.cpp",
		MAME_DIR.. "src/lib/formats/cd90_640_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cgenie_dsk.h,FORMATS["CGENIE_DSK"] = true
--------------------------------------------------

if (FORMATS["CGENIE_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cgenie_dsk.cpp",
		MAME_DIR.. "src/lib/formats/cgenie_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cgen_cas.h,FORMATS["CGEN_CAS"] = true
--------------------------------------------------

if (FORMATS["CGEN_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cgen_cas.cpp",
		MAME_DIR.. "src/lib/formats/cgen_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/coco_cas.h,FORMATS["COCO_CAS"] = true
--------------------------------------------------

if (FORMATS["COCO_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/coco_cas.cpp",
		MAME_DIR.. "src/lib/formats/coco_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/comx35_dsk.h,FORMATS["COMX35_DSK"] = true
--------------------------------------------------

if (FORMATS["COMX35_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/comx35_dsk.cpp",
		MAME_DIR.. "src/lib/formats/comx35_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/concept_dsk.h,FORMATS["CONCEPT_DSK"] = true
--------------------------------------------------

if (FORMATS["CONCEPT_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/concept_dsk.cpp",
		MAME_DIR.. "src/lib/formats/concept_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/coupedsk.h,FORMATS["COUPEDSK"] = true
--------------------------------------------------

if (FORMATS["COUPEDSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/coupedsk.cpp",
		MAME_DIR.. "src/lib/formats/coupedsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/cpis_dsk.h,FORMATS["CPIS_DSK"] = true
--------------------------------------------------

if (FORMATS["CPIS_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/cpis_dsk.cpp",
		MAME_DIR.. "src/lib/formats/cpis_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/csw_cas.h,FORMATS["CSW_CAS"] = true
--------------------------------------------------

if (FORMATS["CSW_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/csw_cas.cpp",
		MAME_DIR.. "src/lib/formats/csw_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/d64_dsk.h,FORMATS["D64_DSK"] = true
--------------------------------------------------

if (FORMATS["D64_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/d64_dsk.cpp",
		MAME_DIR.. "src/lib/formats/d64_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/d71_dsk.h,FORMATS["D71_DSK"] = true
--------------------------------------------------

if (FORMATS["D71_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/d71_dsk.cpp",
		MAME_DIR.. "src/lib/formats/d71_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/d80_dsk.h,FORMATS["D80_DSK"] = true
--------------------------------------------------

if (FORMATS["D80_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/d80_dsk.cpp",
		MAME_DIR.. "src/lib/formats/d80_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/d81_dsk.h,FORMATS["D81_DSK"] = true
--------------------------------------------------

if (FORMATS["D81_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/d81_dsk.cpp",
		MAME_DIR.. "src/lib/formats/d81_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/d82_dsk.h,FORMATS["D82_DSK"] = true
--------------------------------------------------

if (FORMATS["D82_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/d82_dsk.cpp",
		MAME_DIR.. "src/lib/formats/d82_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/dcp_dsk.h,FORMATS["DCP_DSK"] = true
--------------------------------------------------

if (FORMATS["DCP_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/dcp_dsk.cpp",
		MAME_DIR.. "src/lib/formats/dcp_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/dim_dsk.h,FORMATS["DIM_DSK"] = true
--------------------------------------------------

if (FORMATS["DIM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/dim_dsk.cpp",
		MAME_DIR.. "src/lib/formats/dim_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/dip_dsk.h,FORMATS["DIP_DSK"] = true
--------------------------------------------------

if (FORMATS["DIP_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/dip_dsk.cpp",
		MAME_DIR.. "src/lib/formats/dip_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/dmk_dsk.h,FORMATS["DMK_DSK"] = true
--------------------------------------------------

if (FORMATS["DMK_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/dmk_dsk.cpp",
		MAME_DIR.. "src/lib/formats/dmk_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sdf_dsk.h,FORMATS["SDF_DSK"] = true
--------------------------------------------------

if (FORMATS["SDF_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sdf_dsk.cpp",
		MAME_DIR.. "src/lib/formats/sdf_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ep64_dsk.h,FORMATS["EP64_DSK"] = true
--------------------------------------------------

if (FORMATS["EP64_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ep64_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ep64_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/dmv_dsk.h,FORMATS["DMV_DSK"] = true
--------------------------------------------------

if (FORMATS["DMV_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/dmv_dsk.cpp",
		MAME_DIR.. "src/lib/formats/dmv_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/esq16_dsk.h,FORMATS["ESQ16_DSK"] = true
--------------------------------------------------

if (FORMATS["ESQ16_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/esq16_dsk.cpp",
		MAME_DIR.. "src/lib/formats/esq16_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/esq8_dsk.h,FORMATS["ESQ8_DSK"] = true
--------------------------------------------------

if (FORMATS["ESQ8_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/esq8_dsk.cpp",
		MAME_DIR.. "src/lib/formats/esq8_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/excali64_dsk.h,FORMATS["EXCALI64_DSK"] = true
--------------------------------------------------

if (FORMATS["EXCALI64_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/excali64_dsk.cpp",
		MAME_DIR.. "src/lib/formats/excali64_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fc100_cas.h,FORMATS["FC100_CAS"] = true
--------------------------------------------------

if (FORMATS["FC100_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fc100_cas.cpp",
		MAME_DIR.. "src/lib/formats/fc100_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fdd_dsk.h,FORMATS["FDD_DSK"] = true
--------------------------------------------------

if (FORMATS["FDD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fdd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/fdd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/flex_dsk.h,FORMATS["FLEX_DSK"] = true
--------------------------------------------------

if (FORMATS["FLEX_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/flex_dsk.cpp",
		MAME_DIR.. "src/lib/formats/flex_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fm7_cas.h,FORMATS["FM7_CAS"] = true
--------------------------------------------------

if (FORMATS["FM7_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fm7_cas.cpp",
		MAME_DIR.. "src/lib/formats/fm7_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fmsx_cas.h,FORMATS["FMSX_CAS"] = true
--------------------------------------------------

if (FORMATS["FMSX_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fmsx_cas.cpp",
		MAME_DIR.. "src/lib/formats/fmsx_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fmtowns_dsk.h,FORMATS["FMTOWNS_DSK"] = true
--------------------------------------------------

if (FORMATS["FMTOWNS_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fmtowns_dsk.cpp",
		MAME_DIR.. "src/lib/formats/fmtowns_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/fsd_dsk.h,FORMATS["FSD_DSK"] = true
--------------------------------------------------

if (FORMATS["FSD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/fsd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/fsd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/g64_dsk.h,FORMATS["G64_DSK"] = true
--------------------------------------------------

if (FORMATS["G64_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/g64_dsk.cpp",
		MAME_DIR.. "src/lib/formats/g64_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/gtp_cas.h,FORMATS["GTP_CAS"] = true
--------------------------------------------------

if (FORMATS["GTP_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/gtp_cas.cpp",
		MAME_DIR.. "src/lib/formats/gtp_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/guab_dsk.h,FORMATS["GUAB_DSK"] = true
--------------------------------------------------

if (FORMATS["GUAB_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/guab_dsk.cpp",
		MAME_DIR.. "src/lib/formats/guab_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/hector_minidisc.h,FORMATS["HECTOR_MINIDISC"] = true
--------------------------------------------------

if (FORMATS["HECTOR_MINIDISC"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/hector_minidisc.cpp",
		MAME_DIR.. "src/lib/formats/hector_minidisc.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/hect_dsk.h,FORMATS["HECT_DSK"] = true
--------------------------------------------------

if (FORMATS["HECT_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/hect_dsk.cpp",
		MAME_DIR.. "src/lib/formats/hect_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/hect_tap.h,FORMATS["HECT_TAP"] = true
--------------------------------------------------

if (FORMATS["HECT_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/hect_tap.cpp",
		MAME_DIR.. "src/lib/formats/hect_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/hpi_dsk.h,FORMATS["HPI_DSK"] = true
--------------------------------------------------

if (FORMATS["HPI_DSK"]~=null or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/hpi_dsk.cpp",
		MAME_DIR.. "src/lib/formats/hpi_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/hp_ipc_dsk.h,FORMATS["HP_IPC_DSK"] = true
--------------------------------------------------

if (FORMATS["HP_IPC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/hp_ipc_dsk.cpp",
		MAME_DIR.. "src/lib/formats/hp_ipc_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/iq151_dsk.h,FORMATS["IQ151_DSK"] = true
--------------------------------------------------

if (FORMATS["IQ151_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/iq151_dsk.cpp",
		MAME_DIR.. "src/lib/formats/iq151_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/itt3030_dsk.h,FORMATS["ITT3030_DSK"] = true
--------------------------------------------------

if (FORMATS["ITT3030_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/itt3030_dsk.cpp",
		MAME_DIR.. "src/lib/formats/itt3030_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/jvc_dsk.h,FORMATS["JVC_DSK"] = true
--------------------------------------------------

if (FORMATS["JVC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/jvc_dsk.cpp",
		MAME_DIR.. "src/lib/formats/jvc_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/jfd_dsk.h,FORMATS["JFD_DSK"] = true
--------------------------------------------------

if (FORMATS["JFD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/jfd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/jfd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/kaypro_dsk.h,FORMATS["KAYPRO_DSK"] = true
--------------------------------------------------

if (FORMATS["KAYPRO_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/kaypro_dsk.cpp",
		MAME_DIR.. "src/lib/formats/kaypro_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/kc85_dsk.h,FORMATS["KC85_DSK"] = true
--------------------------------------------------

if (FORMATS["KC85_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/kc85_dsk.cpp",
		MAME_DIR.. "src/lib/formats/kc85_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/kc_cas.h,FORMATS["KC_CAS"] = true
--------------------------------------------------

if (FORMATS["KC_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/kc_cas.cpp",
		MAME_DIR.. "src/lib/formats/kc_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/kim1_cas.h,FORMATS["KIM1_CAS"] = true
--------------------------------------------------

if (FORMATS["KIM1_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/kim1_cas.cpp",
		MAME_DIR.. "src/lib/formats/kim1_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/lviv_lvt.h,FORMATS["LVIV_LVT"] = true
--------------------------------------------------

if (FORMATS["LVIV_LVT"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/lviv_lvt.cpp",
		MAME_DIR.. "src/lib/formats/lviv_lvt.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/m20_dsk.h,FORMATS["M20_DSK"] = true
--------------------------------------------------

if (FORMATS["M20_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/m20_dsk.cpp",
		MAME_DIR.. "src/lib/formats/m20_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/m5_dsk.h,FORMATS["M5_DSK"] = true
--------------------------------------------------

if (FORMATS["M5_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/m5_dsk.cpp",
		MAME_DIR.. "src/lib/formats/m5_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/mbee_cas.h,FORMATS["MBEE_CAS"] = true
--------------------------------------------------

if (FORMATS["MBEE_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/mbee_cas.cpp",
		MAME_DIR.. "src/lib/formats/mbee_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/mfm_hd.h,FORMATS["MFM_HD"] = true
--------------------------------------------------

if (FORMATS["MFM_HD"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/mfm_hd.cpp",
		MAME_DIR.. "src/lib/formats/mfm_hd.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/mm_dsk.h,FORMATS["MM_DSK"] = true
--------------------------------------------------

if (FORMATS["MM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/mm_dsk.cpp",
		MAME_DIR.. "src/lib/formats/mm_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ms0515_dsk.h,FORMATS["MS0515_DSK"] = true
--------------------------------------------------

if (FORMATS["MS0515_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ms0515_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ms0515_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/msx_dsk.h,FORMATS["MSX_DSK"] = true
--------------------------------------------------

if (FORMATS["MSX_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/msx_dsk.cpp",
		MAME_DIR.. "src/lib/formats/msx_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/mz_cas.h,FORMATS["MZ_CAS"] = true
--------------------------------------------------

if (FORMATS["MZ_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/mz_cas.cpp",
		MAME_DIR.. "src/lib/formats/mz_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/nanos_dsk.h,FORMATS["NANOS_DSK"] = true
--------------------------------------------------

if (FORMATS["NANOS_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/nanos_dsk.cpp",
		MAME_DIR.. "src/lib/formats/nanos_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/nascom_dsk.h,FORMATS["NASCOM_DSK"] = true
--------------------------------------------------

if (FORMATS["NASCOM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/nascom_dsk.cpp",
		MAME_DIR.. "src/lib/formats/nascom_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/naslite_dsk.h,FORMATS["NASLITE_DSK"] = true
--------------------------------------------------

if (FORMATS["NASLITE_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/naslite_dsk.cpp",
		MAME_DIR.. "src/lib/formats/naslite_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/nes_dsk.h,FORMATS["NES_DSK"] = true
--------------------------------------------------

if (FORMATS["NES_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/nes_dsk.cpp",
		MAME_DIR.. "src/lib/formats/nes_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/nfd_dsk.h,FORMATS["NFD_DSK"] = true
--------------------------------------------------

if (FORMATS["NFD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/nfd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/nfd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/orao_cas.h,FORMATS["ORAO_CAS"] = true
--------------------------------------------------

if (FORMATS["ORAO_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/orao_cas.cpp",
		MAME_DIR.. "src/lib/formats/orao_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/oric_dsk.h,FORMATS["ORIC_DSK"] = true
--------------------------------------------------

if (FORMATS["ORIC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/oric_dsk.cpp",
		MAME_DIR.. "src/lib/formats/oric_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/oric_tap.h,FORMATS["ORIC_TAP"] = true
--------------------------------------------------

if (FORMATS["ORIC_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/oric_tap.cpp",
		MAME_DIR.. "src/lib/formats/oric_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/p6001_cas.h,FORMATS["P6001_CAS"] = true
--------------------------------------------------

if (FORMATS["P6001_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/p6001_cas.cpp",
		MAME_DIR.. "src/lib/formats/p6001_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pasti_dsk.h,FORMATS["PASTI_DSK"] = true
--------------------------------------------------

if (FORMATS["PASTI_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pasti_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pasti_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pc98fdi_dsk.h,FORMATS["PC98FDI_DSK"] = true
--------------------------------------------------

if (FORMATS["PC98FDI_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pc98fdi_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pc98fdi_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pc98_dsk.h,FORMATS["PC98_DSK"] = true
--------------------------------------------------

if (FORMATS["PC98_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pc98_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pc98_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pc_dsk.h,FORMATS["PC_DSK"] = true
--------------------------------------------------

if (FORMATS["PC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pc_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pc_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/phc25_cas.h,FORMATS["PHC25_CAS"] = true
--------------------------------------------------

if (FORMATS["PHC25_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/phc25_cas.cpp",
		MAME_DIR.. "src/lib/formats/phc25_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pk8020_dsk.h,FORMATS["PK8020_DSK"] = true
--------------------------------------------------

if (FORMATS["PK8020_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pk8020_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pk8020_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pmd_cas.h,FORMATS["PMD_CAS"] = true
--------------------------------------------------

if (FORMATS["PMD_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pmd_cas.cpp",
		MAME_DIR.. "src/lib/formats/pmd_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ppg_dsk.h,FORMATS["PPG_DSK"] = true
--------------------------------------------------

if (FORMATS["PPG_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ppg_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ppg_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/primoptp.h,FORMATS["PRIMOPTP"] = true
--------------------------------------------------

if (FORMATS["PRIMOPTP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/primoptp.cpp",
		MAME_DIR.. "src/lib/formats/primoptp.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/pyldin_dsk.h,FORMATS["PYLDIN_DSK"] = true
--------------------------------------------------

if (FORMATS["PYLDIN_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/pyldin_dsk.cpp",
		MAME_DIR.. "src/lib/formats/pyldin_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ql_dsk.h,FORMATS["QL_DSK"] = true
--------------------------------------------------

if (FORMATS["QL_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ql_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ql_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/rk_cas.h,FORMATS["RK_CAS"] = true
--------------------------------------------------

if (FORMATS["RK_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/rk_cas.cpp",
		MAME_DIR.. "src/lib/formats/rk_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/rx50_dsk.h,FORMATS["RX50_DSK"] = true
--------------------------------------------------

if (FORMATS["RX50_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/rx50_dsk.cpp",
		MAME_DIR.. "src/lib/formats/rx50_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sc3000_bit.h,FORMATS["SC3000_BIT"] = true
--------------------------------------------------

if (FORMATS["SC3000_BIT"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sc3000_bit.cpp",
		MAME_DIR.. "src/lib/formats/sc3000_bit.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sf7000_dsk.h,FORMATS["SF7000_DSK"] = true
--------------------------------------------------

if (FORMATS["SF7000_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sf7000_dsk.cpp",
		MAME_DIR.. "src/lib/formats/sf7000_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/smx_dsk.h,FORMATS["SMX_DSK"] = true
--------------------------------------------------

if (FORMATS["SMX_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/smx_dsk.cpp",
		MAME_DIR.. "src/lib/formats/smx_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sol_cas.h,FORMATS["SOL_CAS"] = true
--------------------------------------------------

if (FORMATS["SOL_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sol_cas.cpp",
		MAME_DIR.. "src/lib/formats/sol_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sorc_cas.h,FORMATS["SORC_CAS"] = true
--------------------------------------------------

if (FORMATS["SORC_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sorc_cas.cpp",
		MAME_DIR.. "src/lib/formats/sorc_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sorc_dsk.h,FORMATS["SORC_DSK"] = true
--------------------------------------------------

if (FORMATS["SORC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sorc_dsk.cpp",
		MAME_DIR.. "src/lib/formats/sorc_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/sord_cas.h,FORMATS["SORD_CAS"] = true
--------------------------------------------------

if (FORMATS["SORD_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/sord_cas.cpp",
		MAME_DIR.. "src/lib/formats/sord_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/spc1000_cas.h,FORMATS["SPC1000_CAS"] = true
--------------------------------------------------

if (FORMATS["SPC1000_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/spc1000_cas.cpp",
		MAME_DIR.. "src/lib/formats/spc1000_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/st_dsk.h,FORMATS["ST_DSK"] = true
--------------------------------------------------

if (FORMATS["ST_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/st_dsk.cpp",
		MAME_DIR.. "src/lib/formats/st_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/svi_cas.h,FORMATS["SVI_CAS"] = true
--------------------------------------------------

if (FORMATS["SVI_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/svi_cas.cpp",
		MAME_DIR.. "src/lib/formats/svi_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/svi_dsk.h,FORMATS["SVI_DSK"] = true
--------------------------------------------------

if (FORMATS["SVI_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/svi_dsk.cpp",
		MAME_DIR.. "src/lib/formats/svi_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/tandy2k_dsk.h,FORMATS["TANDY2K_DSK"] = true
--------------------------------------------------

if (FORMATS["TANDY2K_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/tandy2k_dsk.cpp",
		MAME_DIR.. "src/lib/formats/tandy2k_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/thom_cas.h,FORMATS["THOM_CAS"] = true
--------------------------------------------------

if (FORMATS["THOM_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/thom_cas.cpp",
		MAME_DIR.. "src/lib/formats/thom_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/thom_dsk.h,FORMATS["THOM_DSK"] = true
--------------------------------------------------

if (FORMATS["THOM_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/thom_dsk.cpp",
		MAME_DIR.. "src/lib/formats/thom_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/ti99_dsk.h,FORMATS["TI99_DSK"] = true
--------------------------------------------------

if (FORMATS["TI99_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/ti99_dsk.cpp",
		MAME_DIR.. "src/lib/formats/ti99_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/tiki100_dsk.h,FORMATS["TIKI100_DSK"] = true
--------------------------------------------------

if (FORMATS["TIKI100_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/tiki100_dsk.cpp",
		MAME_DIR.. "src/lib/formats/tiki100_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/trd_dsk.h,FORMATS["TRD_DSK"] = true
--------------------------------------------------

if (FORMATS["TRD_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/trd_dsk.cpp",
		MAME_DIR.. "src/lib/formats/trd_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/trs80_dsk.h,FORMATS["TRS80_DSK"] = true
--------------------------------------------------

if (FORMATS["TRS80_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/trs80_dsk.cpp",
		MAME_DIR.. "src/lib/formats/trs80_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/trs_cas.h,FORMATS["TRS_CAS"] = true
--------------------------------------------------

if (FORMATS["TRS_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/trs_cas.cpp",
		MAME_DIR.. "src/lib/formats/trs_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/tvc_cas.h,FORMATS["TVC_CAS"] = true
--------------------------------------------------

if (FORMATS["TVC_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/tvc_cas.cpp",
		MAME_DIR.. "src/lib/formats/tvc_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/tvc_dsk.h,FORMATS["TVC_DSK"] = true
--------------------------------------------------

if (FORMATS["TVC_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/tvc_dsk.cpp",
		MAME_DIR.. "src/lib/formats/tvc_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/tzx_cas.h,FORMATS["TZX_CAS"] = true
--------------------------------------------------

if (FORMATS["TZX_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/tzx_cas.cpp",
		MAME_DIR.. "src/lib/formats/tzx_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/uef_cas.h,FORMATS["UEF_CAS"] = true
--------------------------------------------------

if (FORMATS["UEF_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/uef_cas.cpp",
		MAME_DIR.. "src/lib/formats/uef_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/upd765_dsk.h,FORMATS["UPD765_DSK"] = true
--------------------------------------------------

if (FORMATS["UPD765_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/upd765_dsk.cpp",
		MAME_DIR.. "src/lib/formats/upd765_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/vdk_dsk.h,FORMATS["VDK_DSK"] = true
--------------------------------------------------

if (FORMATS["VDK_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/vdk_dsk.cpp",
		MAME_DIR.. "src/lib/formats/vdk_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/vector06_dsk.h,FORMATS["VECTOR06_DSK"] = true
--------------------------------------------------

if (FORMATS["VECTOR06_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/vector06_dsk.cpp",
		MAME_DIR.. "src/lib/formats/vector06_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/vg5k_cas.h,FORMATS["VG5K_CAS"] = true
--------------------------------------------------

if (FORMATS["VG5K_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/vg5k_cas.cpp",
		MAME_DIR.. "src/lib/formats/vg5k_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/victor9k_dsk.h,FORMATS["VICTOR9K_DSK"] = true
--------------------------------------------------

if (FORMATS["VICTOR9K_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/victor9k_dsk.cpp",
		MAME_DIR.. "src/lib/formats/victor9k_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/vt_cas.h,FORMATS["VT_CAS"] = true
--------------------------------------------------

if (FORMATS["VT_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/vt_cas.cpp",
		MAME_DIR.. "src/lib/formats/vt_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/wd177x_dsk.h,FORMATS["WD177X_DSK"] = true
--------------------------------------------------

if (FORMATS["WD177X_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/wd177x_dsk.cpp",
		MAME_DIR.. "src/lib/formats/wd177x_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/x07_cas.h,FORMATS["X07_CAS"] = true
--------------------------------------------------

if (FORMATS["X07_CAS"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/x07_cas.cpp",
		MAME_DIR.. "src/lib/formats/x07_cas.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/x1_tap.h,FORMATS["X1_TAP"] = true
--------------------------------------------------

if (FORMATS["X1_TAP"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/x1_tap.cpp",
		MAME_DIR.. "src/lib/formats/x1_tap.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/xdf_dsk.h,FORMATS["XDF_DSK"] = true
--------------------------------------------------

if (FORMATS["XDF_DSK"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/xdf_dsk.cpp",
		MAME_DIR.. "src/lib/formats/xdf_dsk.h",
	}
end

--------------------------------------------------
--
--@src/lib/formats/zx81_p.h,FORMATS["ZX81_P"] = true
--------------------------------------------------

if (FORMATS["ZX81_P"]~=null  or _OPTIONS["with-tools"]) then
	files {
		MAME_DIR.. "src/lib/formats/zx81_p.cpp",
		MAME_DIR.. "src/lib/formats/zx81_p.h",
	}
end

end
