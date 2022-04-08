from re import I


def getAverageOfArray(valueArray):
    r = 0
    for i in valueArray:
        r += i
    r = r / len(valueArray)
    return r
