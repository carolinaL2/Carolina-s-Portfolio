from intent_matching import match_intent, handle_small_talk 
from discoverability import handle_discoverability 
from identity_management import handle_identity_management 
from question_answer import handle_question_answering   
from booking_management import show_bookings, handle_booking_creation, handle_booking_edit, show_available_classes, handle_booking_cancellation, show_class_details 

#Handling different intents 
def get_intent_handlers(): 
   
    """ 
    Returns a dictionary mapping intent names to their handler functions. 
    This makes it easy to add new intents without modifying the main logic. 
    """ 
    return { 
        'small_talk': lambda user_input: handle_small_talk(user_input), 
        'discoverability': handle_discoverability, 
        'identity_management': handle_identity_management, 
        'question_answering': lambda user_input: handle_question_answering(user_input), 
        'class_information': show_class_details, 
        'view_bookings': show_bookings, 
        'view_classes': show_available_classes, 
        'booking_creation': handle_booking_creation, 
        'booking_edit': handle_booking_edit, 
        'booking_cancellation': handle_booking_cancellation, 
} 

def handle_unknown_intent(): 
    """ 
    Handles cases where the intent is not recognized - error handling. 
    """ 
    return "I'm not sure I understand. Could you rephrase that?" 


def main(): 

    """ 
    Main chatbot loop that processes user input and generates responses. 
    It also calls functions based on intent being called. 
    """ 
    
    print("Bot: Hi! Welcome to our Fitness Studio! Type 'exit' to end this chat.") 

    #Initialize the intent handlers dictionary 
    intent_handlers = get_intent_handlers() 

    while True: 

        #Get the user input 
        user_input = input("User: ") 

        #Check for exit command, then print a 'Goodbye' message and exit chatbot 
        if user_input.lower() == 'exit': 
            print("Bot: Thank you for using our class booking system. Goodbye!") 
            break 

        response = None 

        #Match the intent of user input 
        intent = match_intent(user_input) 

        #Get the appropriate handler for the intent or use unknown intent handler 
        handler = intent_handlers.get(intent, handle_unknown_intent) 
        
        #Call the handler and get response 
        #If the handler needs user_input, check if it accepts arguments 
        if handler.__code__.co_argcount > 0: 
            response = handler(user_input) 
        else: 
            response = handler() 
       
        #Print the bot's response 
        print("Bot:", response)  

if __name__ == "__main__": 
    main() 