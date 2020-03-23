-- license:MIT
-- copyright-holders:Carl, Patrick Rapin, Reuben Thomas
-- completion from https://github.com/rrthomas/lua-rlcompleter
local exports = {}
exports.name = "console"
exports.version = "0.0.1"
exports.description = "Console plugin"
exports.license = "The BSD 3-Clause License"
exports.author = { name = "Carl" }

local console = exports

function console.startplugin()
	local conth = emu.thread()
	local started = false
	local ln = require("linenoise")
	local preload = false
	local matches = {}
	print("    _/      _/    _/_/    _/      _/  _/_/_/_/");
	print("   _/_/  _/_/  _/    _/  _/_/  _/_/  _/       ");
	print("  _/  _/  _/  _/_/_/_/  _/  _/  _/  _/_/_/    ");
	print(" _/      _/  _/    _/  _/      _/  _/         ");
	print("_/      _/  _/    _/  _/      _/  _/_/_/_/    \n");
	print(emu.app_name() .. " " .. emu.app_version(), "\nCopyright (C) Nicola Salmoria and the MAME team\n");
	print(_VERSION, "\nCopyright (C) Lua.org, PUC-Rio\n");
	-- linenoise isn't thread safe but that means history can handled here
	-- that also means that bad things will happen if anything outside lua tries to use it
	-- especially the completion callback
	ln.historysetmaxlen(10)
	local scr = [[
local ln = require('linenoise')
ln.setcompletion(function(c, str, pos)
	status = str .. "\x01" .. tostring(pos)
	yield()
	ln.addcompletion(c, status:match("([^\x01]*)\x01(.*)"))
end)
return ln.linenoise('\x1b[1;36m[MAME]\x1b[0m> ')
]]
	local keywords = {
		'and', 'break', 'do', 'else', 'elseif', 'end', 'false', 'for',
		'function', 'if', 'in', 'local', 'nil', 'not', 'or', 'repeat',
		'return', 'then', 'true', 'until', 'while'
	}

	-- Main completion function. It evaluates the current sub-expression
	-- to determine its type. Currently supports tables fields, global
	-- variables and function prototype completion.
	local function contextual_list(expr, sep, str, word)
		local function add(value)
			value = tostring(value)
			if value:match("^" .. word) then
				matches[#matches + 1] = value
			end
		end

		-- This function is called in a context where a keyword or a global
		-- variable can be inserted. Local variables cannot be listed!
		local function  add_globals()
			for _, k in ipairs(keywords) do
				add(k)
			end
			for k in pairs(_G) do
				add(k)
			end
		end

		if expr and expr ~= "" then
			local v = loadstring("return " .. expr)
			if v then
				err, v = pcall(v)
				if (not err) or (not v) then
					add_globals()
					return
				end
				local t = type(v)
				if sep == '.' or sep == ':' then
					if t == 'table' then
						for k, v in pairs(v) do
							if type(k) == 'string' and (sep ~= ':' or type(v) == "function") then
								add(k)
							end
						end
					elseif t == 'userdata' then
						for k, v in pairs(getmetatable(v)) do
							if type(k) == 'string' and (sep ~= ':' or type(v) == "function") then
								add(k)
							end
						end
					end
				elseif sep == '[' then
					if t == 'table' then
						for k in pairs(v) do
							if type(k) == 'number' then
								add(k .. "]")
							end
						end
						if word ~= "" then add_globals() end
					end
				end
			end
		end
		if #matches == 0 then
			add_globals()
		end
	end

	local function find_unmatch(str, openpar, pair)
		local done = false
		if not str:match(openpar) then
			return str
		end
		local tmp = str:gsub(pair, "")
		if not tmp:match(openpar) then
			return str
		end
		repeat
			str = str:gsub(".-" .. openpar .. "(.*)", function (s)
				tmp = s:gsub(pair, "")
				if not tmp:match(openpar) then
					done = true
				end
				return s
			end)
		until done or str == ""
		return str
	end

	-- This complex function tries to simplify the input line, by removing
	-- literal strings, full table constructors and balanced groups of
	-- parentheses. Returns the sub-expression preceding the word, the
	-- separator item ( '.', ':', '[', '(' ) and the current string in case
	-- of an unfinished string literal.
	local function simplify_expression(expr, word)
		-- Replace annoying sequences \' and \" inside literal strings
		expr = expr:gsub("\\(['\"])", function (c)
				return string.format("\\%03d", string.byte(c))
			end)
		local curstring
		-- Remove (finished and unfinished) literal strings
		while true do
			local idx1, _, equals = expr:find("%[(=*)%[")
			local idx2, _, sign = expr:find("(['\"])")
			if idx1 == nil and idx2 == nil then
				break
			end
			local idx, startpat, endpat
			if (idx1 or math.huge) < (idx2 or math.huge) then
				idx, startpat, endpat = idx1, "%[" .. equals .. "%[", "%]" .. equals .. "%]"
			else
				idx, startpat, endpat = idx2, sign, sign
			end
			if expr:sub(idx):find("^" .. startpat .. ".-" .. endpat) then
				expr = expr:gsub(startpat .. "(.-)" .. endpat, " STRING ")
			else
				expr = expr:gsub(startpat .. "(.*)", function (str)
						curstring = str
						return "(CURSTRING "
					end)
			end
		end
		-- crop string at unmatched open paran
		expr = find_unmatch(expr, "%(", "%b()")
		expr = find_unmatch(expr, "%[", "%b[]")
		--expr = expr:gsub("%b()"," PAREN ") -- Remove groups of parentheses
		expr = expr:gsub("%b{}"," TABLE ") -- Remove table constructors
		-- Avoid two consecutive words without operator
		expr = expr:gsub("(%w)%s+(%w)","%1|%2")
		expr = expr:gsub("%s+", "") -- Remove now useless spaces
		-- This main regular expression looks for table indexes and function calls.
		return curstring, expr:match("([%.:%w%(%)%[%]_]-)([:%.%[%(])" .. word .. "$")
	end

	local function get_completions(line, endpos)
		matches = {}
		local endstr = line:sub(endpos + 1, -1)
		line = line:sub(1, endpos)
		endstr = endstr or ""
		local start, word = line:match("^(.*[ \t\n\"\\'><=;:%+%-%*/%%^~#{}%(%)%[%].,])(.-)$")
		if not start then
			start = ""
			word = word or line
		else
			word = word or ""
		end

		local str, expr, sep = simplify_expression(line, word)
		contextual_list(expr, sep, str, word)
		if #matches > 1 then
			print("\n")
			for k, v in pairs(matches) do
				print(v)
			end
			return "\x01" .. "-1"
		elseif #matches == 1 then
			return start .. matches[1] .. endstr .. "\x01" .. (#start + #matches[1])
		end
		return "\x01" .. "-1"
	end

	emu.register_periodic(function()
		if conth.yield then
			conth:continue(get_completions(conth.result:match("([^\x01]*)\x01(.*)")))
			return
		elseif conth.busy then
			return
		elseif started then
			local cmd = conth.result
			preload = false
			local func, err = load(cmd)
			if not func then
				if err:match("<eof>") then
					print("incomplete command")
					ln.preload(cmd)
					preload = true
				else
					print("error: ", err)
				end
			else
				local status
				status, err = pcall(func)
				if not status then
					print("error: ", err)
				end
			end
			if not preload then
				ln.historyadd(cmd)
			end
		end
		conth:start(scr)
		started = true
	end)
end

return exports
