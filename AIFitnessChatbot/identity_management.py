from user_data import get_user_name, set_user_name 

def handle_identity_management(): 
    """
    Manages user identity with interactive name setting/changing. 
    """ 
    
    # If the user doesn't have a name set, ask them for it 
    if get_user_name() is None: 
        set_user_name(input("Bot: What's your name?")) 
        return f"Nice to meet you, {get_user_name()}!"  
    
    # Otherwise, ask if user wants to change their username. 
    else: 
        change_name_response = input(
            f"Bot: I have your name set as {get_user_name()}. Do you want to change it? (yes/no)") 

        if change_name_response.lower() == 'yes': 
            set_user_name(input("Bot: What's your new name?")) 
            return f"Great! I'll remember your new name, {get_user_name()}!" 
        else:
            return f"Okay, {get_user_name()}. How can I assist you?" 