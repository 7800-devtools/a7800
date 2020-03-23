local dat = {}
local info, ver
local datread = require("data/load_dat")
do
	local convert = require("data/button_char")
	datread, ver = datread.open("command.dat", "# Command List%-Shorthand", convert)
end

function dat.check(set, softlist)
	if softlist or not datread then
		return nil
	end
	local status
	status, info = pcall(datread, "cmd", "info", set)
	if not status or not info then
		return nil
	end
	info = "#jf\n" .. info
	return "Command"
end

function dat.get()
	return info
end

return dat
