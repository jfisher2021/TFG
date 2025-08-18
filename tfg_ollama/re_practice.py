import re 
import math

bad_text = """
0.000: (start_welcome) [1.000]  
1.000: (move home dibejo) [15.000]  
16.000: (explain dibejo) [15.000]  
31.000: (move dibejo elgrito) [15.000]  
46.000: (explain elgrito) [15.000]  
61.000: (move elgrito home) [15.000]  
76.000: (recharge) [5.000]  
81.000: (move home guernica) [15.000]  
96.000: (explain guernica) [15.000]  
111.000: (move guernica monalisa) [15.000]  
126.000: (explain monalisa) [15.000]  
141.000: (move monalisa nocheestrellada) [15.000]
"""

good_text = """
0.000: (start_welcome tiago)  [1.000]
1.001: (move tiago home dibejo)  [15.000]
16.002: (explain_painting tiago dibejo)  [15.000]
31.002: (move tiago dibejo elgrito)  [15.000]
46.002: (explain_painting tiago elgrito)  [15.000]
61.002: (move tiago elgrito home)  [15.000]
76.002: (recharge tiago home)  [5.000]
81.003: (move tiago home guernica)  [15.000]
96.003: (explain_painting tiago guernica)  [15.000]
111.003: (move tiago guernica monalisa)  [15.000]
126.002: (recharge tiago home)  [5.000]
126.003: (explain_painting tiago monalisa)  [15.000]
141.003: (move tiago monalisa nocheestrellada)  [15.000]
"""
new_text = """ 0.000: (start_welcome tiago) [1.000]  \n1.001: (move tiago home dibejo) [15.000]\
    \  \n16.002: (explain_painting tiago dibejo) [15.000]  \n31.002: (move tiago\
    \ dibejo elgrito) [15.000]  \n46.002: (explain_painting tiago elgrito)\
    \ [15.000]  \n61.002: (move tiago elgrito home) [15.000]  \n76.003: (recharge\
    \ tiago home) [5.000]  \n81.003: (move tiago home guernica) [15.000]  \n96.004:\
    \ (explain_painting tiago guernica) [15.000]  \n111.004: (move tiago guernica\
    \ monalisa) [15.000]  \n126.004: (explain_painting tiago monalisa) [15.000]\
    \  \n141.004: (move tiago monalisa home) [15.000]  \n156.005: (recharge tiago\
    \ home) [5.000]  \n161.005: (move tiago home nocheestrellada) [15.000]  \n\
    176.006: (explain_painting tiago nocheestrellada) [15.000]
    """
# Define the text to search

def validate_battery(text):
    moves= []
    explain_painting = []
    recharge = []
    battery = 100
    # Define the regex pattern to match the lines
    explain_painting_pattern = r"(\d+\.\d+): \((explain_painting.*?)\)  \[(.*?)\]"
    move_pattern = r"(\d+\.\d+): \((move.*?)\)  \[(.*?)\]"
    recharge_pattern = r"(\d+\.\d+): \((recharge.*?)\)  \[(.*?)\]"
    # Find all matches in the text
    # Print the matches
    matches_explain = re.findall(explain_painting_pattern, text)
    move_matches = re.findall(move_pattern, text)
    recharge_matches = re.findall(recharge_pattern, text)
    # Print the matches
    for i, match in enumerate(matches_explain):
        explain_painting.append(match[0])
        
    for i, match in enumerate(recharge_matches):
        recharge.append(match[0])

    for i, match in enumerate(move_matches):
        moves.append(match[0])
    for i_recharge in range(len(recharge_matches)):

        for i, time in enumerate(moves):
            if float(time) < float(recharge[i_recharge]):
                print(time, recharge[i_recharge])
                battery -= 20
                moves[i] = math.inf

        for i, time in enumerate(explain_painting):
            if float(time) < float(recharge[i_recharge]):
                battery -= 10
                explain_painting[i] = math.inf

        if battery < 0:
            return False
        battery = 100
    return True

def validate_format(text):
    start_phrase = r"(\d+\.\d+: \(start_welcome tiago\).*)"
    move_phrase = r"\d+\.\d+: \(move tiago [a-z]+ [a-z]+_[a-z]+\)  \[\d+\.\d+\]"
    explain_painting_phrase = r"\d+\.\d+: .*\(explain_painting tiago [a-z]+_[a-z]+\)  \[\d+\.\d+\]"
    recharge_phrase = r"\d+\.\d+: .*\(recharge tiago [a-z]+\)  \[\d+.*]"
    matches_start = re.findall(start_phrase, text)
    print(matches_start)
    if not matches_start:
        print("The text does not start with the welcome phrase.")
        return False
    matches_move = re.findall(move_phrase, text)
    print(matches_move)
    if not matches_move:
        print("The text does not contain the move phrase.")
        return False
    matches_explain = re.findall(explain_painting_phrase, text)
    print(matches_explain)
    if not matches_explain:
        print("The text does not contain the explain painting phrase.")
        return False
    matches_recharge = re.findall(recharge_phrase, text)
    print(matches_recharge)
    if not matches_recharge:
        print("The text does not contain the recharge phrase.")
        return False
    return True


# is_valid = validate_battery(good_text)
# if is_valid:
#     print("Battery is sufficient.")
# else:
#     print("Battery is empty, need to recharge!")

# is_valid_format = validate_format(good_text)
# if is_valid_format:
#     print("The text format is valid.")
# else:
#     print("The text format is invalid.")


is_valid = validate_battery(new_text)
if is_valid:
    print("Battery is sufficient.")
else:
    print("Battery is empty, need to recharge!")

is_valid_format = validate_format(new_text)
if is_valid_format:
    print("The text format is valid.")
else:
    print("The text format is invalid.")