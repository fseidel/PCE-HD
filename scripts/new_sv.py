


def run(name, description):
  
  output = "";
  output += "`default_nettype none\n"
  output += "\n"
  output += "/*\n"
  output += " * " + description + "\n"
  output += " *\n"
  output += " * (C) 2018 Ford Seidel and Amolak Nagi\n"
  output += " */\n"
  output += "\n"
  output += "module " + name + "();\n"
  output += "\n"
  output += "endmodule: " + name + "\n"

  filename = name + ".sv"

  f = open(filename, 'w')
  f.write(output)
  f.close()
 
  print("New file named " + filename + " created") 


if __name__ == '__main__':
  from sys import argv
  if (len(argv) <= 2):
    print("Format: ModuleName \"Description\"\n")
    print("Note: The Description must be in quotes")
  else:
    run(argv[1], argv[2])

