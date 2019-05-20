import json

def parse(f):
	"""
	Output is in format:
	[cmd1, cmd2, cmd3, etc]
	Cmds are in format:
	[frame, command, doJumpId, autoContinue, params[0], params[1], ... params[6]]
	"""
	file = open(f, "r")
	cmds = json.loads(file.read())
	itemsList = cmds["mission"]["items"]
	out = []
	for i in itemsList:
		out.append((
			0,
			0,
			0,
			i["frame"],
			i["command"],
			i["doJumpId"],
			i["autoContinue"],
			i["params"][0],
			i["params"][1],
			i["params"][2],
			i["params"][3],
			i["params"][4],
			i["params"][5],
			i["params"][6]
		))
	file.close()
	return(out)