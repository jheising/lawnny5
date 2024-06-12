import json

import pytweening
import copy
import uuid


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
    normalized_element = {"id": uuid.uuid4().hex, "position": position_key_position, "type": element[0], "msg": element[1]}
    if len(element) >= 3:
        normalized_element["tween"] = element[2]
    return normalized_element


def interpolate_keyframe_element(from_element, to_element, tween_type, percentage_done):
    from_percentage = from_element["position"]
    to_percentage = to_element["position"]

    # convert our total percentage in our keyframes into a relative percentage between the two elements
    tween_percentage = (percentage_done - from_percentage) / (to_percentage - from_percentage)
    tween_function = getattr(pytweening, tween_type)

    interpolated_element = copy.copy(from_element)
    interpolated_element["msg"] = tween_object_values(from_element["msg"], to_element["msg"], tween_function, tween_percentage)

    return interpolated_element


class KeyframeInterpreter:
    def __init__(self, keyframes, total_duration):

        self.keyframes = copy.deepcopy(keyframes)
        self.total_duration = total_duration

        # Convert all of our keyframe position strings into percentages
        current_position = 0
        self.keyframe_positions = []
        for keyframe in self.keyframes:

            keyframe_position_string = keyframe[0]

            if "%" in keyframe_position_string:
                keyframe_position_value = float(keyframe_position_string.replace("%", "")) / 100.0
            elif "+" in keyframe_position_string:
                keyframe_position_value = (float(keyframe_position_string.replace("+", "")) / total_duration) + current_position
            else:
                keyframe_position_value = float(keyframe_position_string) / total_duration

            current_position = keyframe_position_value
            keyframe[0] = keyframe_position_value

            self.keyframe_positions.append(keyframe_position_value)

        self.keyframe_positions = sorted(self.keyframe_positions)

        self.normalized_keyframes = []
        self.once_element_ids = []

        for keyframe_position in self.keyframe_positions:
            elements = []

            for keyframe in self.keyframes:
                if keyframe[0] == keyframe_position:
                    elements = elements + keyframe[1:]

            normalized_elements = list(map(lambda element: normalize_element(element, keyframe_position), elements))

            normalized_element_dict = {
                "position": keyframe_position,
                "elements": {}
            }

            for normalized_element in normalized_elements:
                normalized_element_dict["elements"][normalized_element["type"]] = normalized_element

            self.normalized_keyframes.append(normalized_element_dict)

    def restart(self):
        self.once_element_ids = []

    def play(self, percentage_done):
        elements = self._get_elements_at_position(percentage_done)

        return list(map(lambda element: {"type": element["type"], "msg": element["msg"]}, elements))

    def _get_elements_at_position(self, percentage_done):
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
            tween_type = to_element.get("tween")

            # If this element is transitioning from a previous element, we'll need to interpolate it
            if tween_type and tween_type != "once" and from_element:
                interpolated_element = interpolate_keyframe_element(from_element, to_element, tween_type, percentage_done)
                current_element = interpolated_element
            elif from_element and percentage_done < to_element["position"]:
                current_element = from_element
            elif not from_element and percentage_done < to_element["position"]:
                continue
            else:
                current_element = to_element

            if current_element.get("tween") == "once":
                element_id = current_element["id"]
                # If this once element has already been sent, then don't bother sending it again
                if element_id in self.once_element_ids:
                    continue
                else:
                    self.once_element_ids.append(element_id)

            return_elements.append(current_element)

        return return_elements
