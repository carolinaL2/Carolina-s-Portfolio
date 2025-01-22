# Global variables to track user and booking information   
user_name = None 
booking = None 

def get_user_name(): 

    """
    Getter: Retrieves the current user's name. 
    """ 

    return user_name 

def set_user_name(name): 

    """
    Setter: Sets the new user's name. 
    """ 

    global user_name 
    user_name = name 

def get_booking(): 

    """
    Getter: Retrieves the current booking information. 
    """ 
    
    return booking 

def set_booking(booking_details): 

    """
    Setter: Sets the new booking information. 
    """ 

    global booking 
    booking = booking_details 