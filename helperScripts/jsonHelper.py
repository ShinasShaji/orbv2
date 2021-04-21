import json


def dictToJson(dict, path):
    """Exports given dictionary as a .json file"""
    with open(path, "w") as jsonFile:
        json.dump(dict, jsonFile, sort_keys=True, indent=4)
        
    print("Exported as", path)


def jsonToDict(path):
    """Reads given json from path and returns contents as a dictionary"""
    with open(path, "r") as jsonFile:
        dict = json.load(jsonFile)
        
    print("Reading from", path)

    return dict



if __name__=="__main__":
    print("Import to use")
