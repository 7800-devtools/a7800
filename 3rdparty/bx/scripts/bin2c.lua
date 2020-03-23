--
-- Copyright 2010-2017 Branimir Karadzic. All rights reserved.
-- License: https://github.com/bkaradzic/bx#license-bsd-2-clause
--

project "bin2c"
	kind "ConsoleApp"

	includedirs {
		"../include",
	}

	files {
		"../tools/bin2c/**.cpp",
		"../tools/bin2c/**.h",
	}

	links {
		"bx",
	}

	configuration { "mingw-*" }
		targetextension ".exe"

	configuration { "linux-*" }
		links {
			"pthread",
		}

	configuration {}

	strip()
