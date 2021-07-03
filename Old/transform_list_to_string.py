def transform_list_to_string(list_to_transform):
    """
    Simple method to transform a list of strings to a single string with each of the list elements separated by comma
    :param list_to_transform: list of strings
    :return:
    """
    headers_str = ''
    for i in range(len(list_to_transform)):
        if i != 0:
            header = ',' + list_to_transform[i]
        else:
            header = list_to_transform[i]
        headers_str += header

    return headers_str
