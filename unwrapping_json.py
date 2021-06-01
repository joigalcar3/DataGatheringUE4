def unwrapping_json(branch, output, keys_values, predecessor='', unwrapped_info=None):
    """
    Recursive method that unwraps all the information stored by the AirSim tree data structure to a list. This is done
    such that the information can be later stored in .csv format
    :param branch: list of the keys corresponding to the variables stored in the next level deep of the AirSim tree, and
    that should be 'investigated' in the next recursion
    :param output: branch within the AirSim data structure that is next to be unfolded
    :param keys_values: whether the keys or the values are to be stored
    :param predecessor: for a header (key) n levels deep in the data structure, it is saved as follows:
    first_level.second_level.____.nth_level. In order to achieve this notation, the function is passed the state of the
    naming up until the previous level. Therefore, for the third level, the predecessor would be:
    first_level.second_level.
    :param unwrapped_info: the list with the information that will be returned by this method
    :return:
    """
    # Initialise the list to stored the output
    if unwrapped_info is None:
        unwrapped_info = []

    # For each of the keys in the current branch of the tree
    for element in branch:
        try:
            # The list of the keys that should be 'investigated' in the next recursion
            branch_recursive = list(getattr(output, element).__dict__.keys())

            # The state of the tree in the current branch
            output_recursive = getattr(output, element)

            # Current naming of the headers
            predecessor_recursive = predecessor + element + '.'

            unwrapped_info_deeper = unwrapping_json(branch_recursive, output_recursive, keys_values,
                                                    predecessor_recursive)

            # Store the output information provided by deeper layers of the tree
            unwrapped_info += unwrapped_info_deeper
        except:
            if keys_values == 'keys':
                # In the case that header information should be stored
                predecessor_recursive = predecessor + element
                unwrapped_info.append(predecessor_recursive)
            elif keys_values == 'values':
                # In the case that the data should be stored
                data = getattr(output, element)

                # Translate booleans to integers
                if type(data) == bool:
                    data = int(data)

                # Remove empty lists from the output
                elif type(data) == list:
                    data = -1
                unwrapped_info.append(data)
    return unwrapped_info
