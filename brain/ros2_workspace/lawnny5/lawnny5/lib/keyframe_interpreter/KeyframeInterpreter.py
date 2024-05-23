import pytweening
import copy
import json

# Convert something like 10% into 0.1
def get_keyframe_key_value(key):
    key = key.replace("%", "")
    return float(key) / 100.0


def is_number(value):
    return isinstance(value, (int, float))


def tween_object_values(start_object, end_object, tween_function, tween_percentage):
    tween_percentage = min(1.0, max(0.0, tween_percentage))

    tween_value = tween_function(tween_percentage)

    object1_keys = start_object.keys()
    object2_keys = end_object.keys()
    result = {}

    for key in object2_keys:

        obj2_value = end_object[key]
        result[key] = obj2_value

        if key in object1_keys:
            obj1_value = start_object[key]

            if isinstance(obj1_value, dict) and isinstance(obj2_value, dict):
                result[key] = tween_object_values(obj1_value, obj2_value, tween_function, tween_percentage)
            elif is_number(obj1_value) and is_number(obj2_value):
                result[key] = obj1_value + ((obj2_value - obj1_value) * tween_value)

    return result


def normalize_element(element, position_key_position):
    normalized_element = {"position": position_key_position, "type": element[0], "msg": element[1]}
    if len(element) >= 3:
        normalized_element["tween"] = element[2]
    return normalized_element


def interpolate_keyframe_element(from_element, to_element, percentage_done):
    tween_type = to_element.get("tween")

    # if there is no tween to, then there is no need to interpolate
    if not tween_type:
        return to_element

    from_percentage = from_element["position"]
    to_percentage = to_element["position"]

    # convert our total percentage in our keyframes into a relative percentage between the two elements
    tween_percentage = (percentage_done - from_percentage) / (to_percentage - from_percentage)
    tween_function = getattr(pytweening, tween_type)

    interpolated_element = copy.copy(from_element)
    interpolated_element["msg"] = tween_object_values(from_element["msg"], to_element["msg"], tween_function, tween_percentage)

    return interpolated_element


class KeyframeInterpreter:
    def __init__(self, keyframes):

        self.keyframes = copy.deepcopy(keyframes)

        # Sort our keyframes into their proper order
        self.keyframe_position_keys = sorted(map(lambda key: [key, get_keyframe_key_value(key)], keyframes.keys()), key=lambda value: value[1])

        self.normalized_keyframes = []
        self.last_message_strings = []

        for position_key_info in self.keyframe_position_keys:
            position_key = position_key_info[0]
            position_key_position = position_key_info[1]

            elements = self.keyframes[position_key]
            normalized_elements = list(map(lambda element: normalize_element(element, position_key_position), elements))

            normalized_element_dict = {
                "position": position_key_position,
                "elements": {}
            }

            for normalized_element in normalized_elements:
                normalized_element_dict["elements"][normalized_element["type"]] = normalized_element

            self.normalized_keyframes.append(normalized_element_dict)

    def restart(self):
        self.last_message_strings = []

    def play(self, percentage_done):
        new_messages = self.get_messages_at_position(percentage_done)

        # Remove any messages that haven't changed since our last run through
        new_message_strings = list(map(lambda msg: json.dumps(msg, sort_keys=True), new_messages))

        return_messages = []

        for message_index in range(len(new_message_strings)):
            new_message_string = new_message_strings[message_index]
            if new_message_string not in self.last_message_strings:
                return_messages.append(new_messages[message_index])

        self.last_message_strings = new_message_strings

        return return_messages

    def get_messages_at_position(self, percentage_done):
        current_keyframe_element_state = {}
        prev_keyframe_element_state = {}
        return_elements = []

        for keyframe in self.normalized_keyframes:
            cur_position = keyframe["position"]

            elements = keyframe.get("elements")

            if not elements:
                break

            for element in elements.values():
                element_type = element["type"]

                prev_element = current_keyframe_element_state.get(element_type)

                if prev_element:
                    prev_keyframe_element_state[element_type] = prev_element

                current_keyframe_element_state[element_type] = element

            # If the position of this keyframe is at or further than our current position then we should stop processing future keyframes
            if cur_position >= percentage_done:
                break

        for to_element in current_keyframe_element_state.values():
            from_element = prev_keyframe_element_state.get(to_element["type"])

            # If this element is transitioning from a previous element, we'll need to interpolate it
            if from_element:
                interpolated_element = interpolate_keyframe_element(from_element, to_element, percentage_done)
                current_element = {"type": interpolated_element["type"], "msg": interpolated_element["msg"]}
            else:
                current_element = {"type": to_element["type"], "msg": to_element["msg"]}

            return_elements.append(current_element)

        return return_elements
