def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass
 
    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass
 
    return False

if __name__ == "__main__":
    file_name = '/home/max/OneDrive/Research Projects/SickKids/STL/Ascii STL/Concave_Pyramid.stl'

    # Using readline() 
    concave_file = open(file_name, 'r') 
    convex_file = open(file_name.replace('Concave','Convex_clean'),'w')
    
    while True: 
        # Get next line from file 
        line = concave_file.readline() 
    
        # if line is empty 
        # end of file is reached 
        if not line: 
            break
        
        if "facet normal" in line:
            normals = [-float(s) for s in line.split() if is_number(s)]
            line = "facet normal {} {} {}\n".format(normals[0], normals[1], normals[2]) 
        convex_file.write(line)
    
    concave_file.close() 