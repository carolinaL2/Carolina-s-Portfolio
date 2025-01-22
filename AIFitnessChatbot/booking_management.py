from datetime import datetime, timedelta
from user_data import get_booking, get_user_name, set_booking, set_user_name
import csv
from fitness_classes import FitnessClasses

# Initialize FitnessClasses instance
fitness_classes = FitnessClasses()

# Initialize class schedule
class_schedule = None

def get_schedule():
    global class_schedule
    if class_schedule is None:
        class_schedule = _generate_schedule()
    return class_schedule 

def get_date():
    while True:
        try:
            new_date = input("Bot: Enter a class booking date(DD-MM-YYYY): ")
            datetime.strptime(new_date, '%d-%m-%Y')
            return new_date
        except ValueError:
            print("Bot: Invalid date format. Please use DD-MM-YYYY.")

def get_time():
    while True:
        try:
            new_time = input("Bot: Enter the class booking time (HH:MM): ")
            datetime.strptime(new_time, '%H:%M')
            return new_time
        except ValueError:
            print("Bot: Invalid time format. Please use HH:MM.")

def _generate_schedule():
    schedule = {}
    start_date = datetime.now()
    times = ['07:00', '09:00', '11:00', '13:00', '15:00', '17:00', '19:00', '21:00']
    class_types = fitness_classes.get_class_names()
    instructors = ['Sarah', 'Mike', 'Emma', 'John', 'Marie', 'Tim']

    for i in range(7):
        date = (start_date + timedelta(days=i)).strftime('%d-%m-%Y')
        daily_classes = {}

        for class_idx, class_type in enumerate(class_types):
            time_idx1 = (class_idx * 2) % len(times)
            time_idx2 = (class_idx * 2 + 1) % len(times)
            class_times = [times[time_idx1], times[time_idx2]]

            for time in class_times:
                slot = f"{date} {time}"
                instructor_idx = (i + time_idx1) % len(instructors)
                daily_classes[slot] = {
                    'type': class_type,
                    'available_spots': fitness_classes.get_class_capacity(class_type),
                    'booked_users': [],
                    'instructor': instructors[instructor_idx]
                }
        schedule[date] = daily_classes
    return schedule

def show_available_classes(date, has_date=True, return_list=False):
    if has_date:
        date = get_date()

    current_schedule = get_schedule()

    if date not in current_schedule:
        if return_list:
            return []
        return "No classes available for this date. Please choose a date within the next 7 days."

    for slot, details in current_schedule[date].items():
        class_type = details['type']
        booking_time = slot.split()[1]
        current_bookings = 0

        try:
            with open('bookings.csv', 'r') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    if len(row) >= 4:
                        if (row[1].strip() == date and 
                            row[2].strip() == booking_time and 
                            row[3].strip().lower() == class_type.lower()):
                            current_bookings += 1

            details['available_spots'] = fitness_classes.get_class_capacity(class_type) - current_bookings

        except FileNotFoundError:
            details['available_spots'] = fitness_classes.get_class_capacity(class_type)

    available_classes = []
    print(f"\nAvailable Classes for {date}:")
    print("=" * (len(date) + 20))

    for slot, details in current_schedule[date].items():
        if details['available_spots'] > 0:
            class_type = details['type']
            class_info = fitness_classes.get_class(class_type)
            available_classes.append({
                "time": slot.split()[1],
                "type": class_type,
                "instructor": details['instructor'],
                "available_spots": details['available_spots'],
                "price": class_info['price']
            })

            print(
                f"\nTime: {slot.split()[1]}"
                f"\nClass: {class_info['name']}"
                f"\nInstructor: {details['instructor']}"
                f"\nDuration: {class_info['duration']}"
                f"\nIntensity: {class_info['intensity']}"
                f"\nAvailable spots: {details['available_spots']}/{class_info['capacity']}"
                f"\nPrice: ${class_info['price']}"
                f"\n{'-' * 50}"
            )

    if not available_classes:
        print("No available classes for this date.")
    else: 
        print("To book a class, use the 'book class' command.")

    return available_classes if return_list else None

def get_workoutClass(available_classes):
    while True:
        selected_time = get_time()
        
        for class_option in available_classes:
            if class_option['time'] == selected_time:
                return class_option

        print("Bot: Invalid time. Please enter a time listed in the available classes.")

def show_class_details():
    print("\nAvailable class types:")
    for class_type in fitness_classes.get_class_names():
        print(f"- {class_type}")

    while True:
        user_input = input("\nBot: Which class would you like to learn more about?: ").lower()
        
        if fitness_classes.class_exists(user_input):
            class_info = fitness_classes.get_class(user_input)
            break
        else:
            print("Bot: Invalid class type. Please choose from the available classes listed above.")

    benefits_list = ", ".join(class_info['benefits'])
    
    print(f"\n{class_info['name']} Class Details:")
    print("=" * (len(class_info['name']) + 13))
    print(f"Description: {class_info['description']}")
    print(f"Duration: {class_info['duration']}")
    print(f"Price: ${class_info['price']}")
    print(f"Maximum Capacity: {class_info['capacity']} people")
    print(f"Intensity Level: {class_info['intensity']}")
    print(f"Benefits: {benefits_list}")
    print(f"Suitable for: {class_info['suitable_for']}")
    print("\nTo book this class, use the 'book class' command.")

def is_booking_available(booking_date, booking_time, class_type):
    if not fitness_classes.class_exists(class_type):
        raise ValueError(f"Invalid workout class: {class_type}")

    schedule = get_schedule()
    if booking_date not in schedule:
        return False

    time_slot = f"{booking_date} {booking_time}"
    if time_slot not in schedule[booking_date]:
        return False

    class_slot = schedule[booking_date][time_slot]
    if class_slot['type'] != class_type:
        return False

    class_capacity = fitness_classes.get_class_capacity(class_type)
    current_bookings = 0

    try:
        with open('bookings.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            
            for row in reader:
                if len(row) >= 4:
                    if (row[1].strip() == booking_date and 
                        row[2].strip() == booking_time and 
                        row[3].strip().lower() == class_type.lower()):
                        current_bookings += 1

                        if current_bookings >= class_capacity:
                            return False

        class_slot['available_spots'] = class_capacity - current_bookings
        return True

    except FileNotFoundError:
        return True

# Appends a row "booking_details" into the bookings csv file 
def save_booking(booking_details):
    # Convert the booking details to a list in the correct order
    booking_row = [
        booking_details['user'],
        booking_details['date'],
        booking_details['time'],
        booking_details['w_class']
    ]
    
    with open('bookings.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(booking_row)

    csvfile.close() 

def show_bookings(): 
    """
    Shows the user's current booking and any previous bookings.
    Returns a formatted string containing the booking information.
    """
    user_name = get_user_name()
    if not user_name:
        return "Please set your name first using the 'set name' command."

    current_booking = get_booking()
    all_bookings = []
    
    try:
        with open('bookings.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 4 and row[0].strip().lower() == user_name.lower():
                    booking = {
                        'user': row[0],
                        'date': row[1],
                        'time': row[2],
                        'w_class': row[3]
                    }
                    all_bookings.append(booking)
    except FileNotFoundError:
        if not current_booking:
            return f"No bookings found for {user_name}."

    # Prepare the output message
    output_messages = []
    
    # Show current booking if it exists
    if current_booking:
        output_messages.append("Your current booking:")
        output_messages.append(f"- {current_booking['w_class']} class on {current_booking['date']} at {current_booking['time']}")
    
    # Show previous bookings if they exist
    previous_bookings = [b for b in all_bookings if not (current_booking and 
                        b['date'] == current_booking['date'] and 
                        b['time'] == current_booking['time'] and 
                        b['w_class'].lower() == current_booking['w_class'].lower())]
    
    if previous_bookings:
        if output_messages:
            output_messages.append("\nYour previous bookings:")
        else:
            output_messages.append("Your previous bookings:")
        
        for booking in previous_bookings:
            output_messages.append(f"- {booking['w_class']} class on {booking['date']} at {booking['time']}")
    
    if not output_messages:
        return f"No bookings found for {user_name}."
    
    return "\n".join(output_messages) 

# Output the user's current booking
def output_booking(): 
    x = get_booking()  
    name = x['user'] 
    date = x['date'] 
    time = x['time'] 
    w_class = x['w_class'] 

    return f"Hi {name}, your current booking is for {time} on {date}, for {w_class}." 

def check_existing_booking_from_csv(user_name):
    """
    Check if a user has any existing bookings in the CSV file.
    Returns the booking details if found, None otherwise.
    """
    try:
        with open('bookings.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 4 and row[0].strip().lower() == user_name.lower():
                    return {
                        'user': row[0],
                        'date': row[1],
                        'time': row[2],
                        'w_class': row[3]
                    }
    except FileNotFoundError:
        return None
    return None 

def confirm_booking_action(prompt):
    """
    Generic confirmation helper that handles yes/no responses
    Returns True if user confirms, False otherwise
    """
    while True:
        response = input(f"Bot: {prompt} (yes/no): ").lower().strip()
        if response in ['yes', 'y']:
            return True
        elif response in ['no', 'n']:
            return False
        else:
            print("Bot: Please answer with 'yes' or 'no'") 

def handle_booking_creation(): 
    # If the user hasn't set a name, ask them for one 
    user_name = get_user_name()
    if not user_name:
        user_name = input("Bot: It seems I don't have your name! Please enter your name for the booking: ")
        set_user_name(user_name) 

    existing_booking = check_existing_booking_from_csv(user_name) 
    if existing_booking:
        print(f"Bot: I found an existing booking for {existing_booking['w_class']} on {existing_booking['date']} at {existing_booking['time']}")
        edit_response = input("Bot: Would you like to edit this booking instead? (yes/no): ")
        
        if edit_response.lower() == 'yes':
            set_booking(existing_booking)  # Set the existing booking so edit function can access it
            return handle_booking_edit()  # Redirect to edit flow
        elif edit_response.lower() != 'no':
            return "Bot: Keeping your existing booking. Let me know if you want to make any changes!"

    print(f"Bot: Let's get your booking made, {get_user_name()}") 

    booking_date = get_date()
    available_classes = show_available_classes(booking_date, False, True)

    if not available_classes:
        return "Bot: No available classes for this date."

    selected_class = get_workoutClass(available_classes)
    if not selected_class:
        return "Bot: Unable to select a valid class time."

    booking_time = selected_class['time']
    workout_class = selected_class['type'] 
    user_name = get_user_name() 

    booking_summary = f"""
        Bot: Here's your booking summary:
        - Class: {workout_class}
        - Date: {booking_date}
        - Time: {booking_time}
        - Price: ${selected_class['price']}
        - Instructor: {selected_class['instructor']}
        """ 
    print(booking_summary)
    
    if not confirm_booking_action("Would you like to confirm this booking?"): 
        return "Bot: Booking cancelled. Let me know if you'd like to try booking a different class!" 

    # Check if the booking is available and proceed to confirm it 
    if is_booking_available(booking_date, booking_time, workout_class):
        booking_details = {
            'user': user_name,
            'date': booking_date,
            'time': booking_time,
            'w_class': workout_class 
        }

        save_booking(booking_details)
        set_booking(booking_details)
        return f"Booking created for {user_name} on {booking_date} at {booking_time} for {workout_class}." 
    else:
        return "Bot: Sorry, this time slot is already booked. Please choose a different time."


def handle_booking_edit(): 
    current_booking = get_booking() 
    
    # Check if the user has an existing booking
    if current_booking is None:
        return "You don't have an existing booking. Please create a booking first." 
    
    # Confirm with the user if they want to edit their booking 
    if not confirm_booking_action("You want to edit your booking, is that correct?"):
        print("Bot:", output_booking())
        return f"Okay {get_user_name()}, your booking will remain unchanged!" 

    # Prompt for new booking details
    new_date = get_date()
    available_classes = show_available_classes(new_date, False, True)
    if not available_classes:
        return "No available classes on the selected date."
    
    selected_class = get_workoutClass(available_classes)
    new_time = selected_class['time']
    new_w_class = selected_class['type'] 

    # Confirm changes before proceeding
    change_summary = f"""
        Bot: Here are your proposed changes:
        Original booking:
        - Class: {current_booking['w_class']}
        - Date: {current_booking['date']}
        - Time: {current_booking['time']}

        New booking:
        - Class: {new_w_class}
        - Date: {new_date}
        - Time: {new_time}
        """ 
    print(change_summary) 

    if not confirm_booking_action("Would you like to confirm these changes?"):
        return "Bot: Changes cancelled. Your original booking remains unchanged."

    # Update booking if confirmed 
    # Read the CSV file to update the booking
    rows_to_write = []
    booking_updated = False
    
    with open('bookings.csv', mode='r', newline='') as csvfile:
        reader = csv.reader(csvfile)  # Changed to basic reader since we know the format
        
        for row in reader: 
            # Compare the exact values from the CSV
            if (row[0].strip() == current_booking['user'].strip() and 
                row[1].strip() == current_booking['date'].strip() and 
                row[2].strip() == current_booking['time'].strip() and 
                row[3].strip().lower() == current_booking['w_class'].strip().lower()): 
                
                # Check if the new slot is available
                if not is_booking_available(new_date, new_time, new_w_class):
                    return f"Sorry {get_user_name()}, this slot is already booked. Please try a different time."
                
                # Add the updated booking
                rows_to_write.append([
                    current_booking['user'],
                    new_date,
                    new_time,
                    new_w_class
                ])
                booking_updated = True
            else:
                rows_to_write.append(row)

    # Write the updated data back to the CSV file
    with open('bookings.csv', mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(rows_to_write)

    # Update the in-memory booking
    updated_booking = {
        'user': current_booking['user'],
        'date': new_date,
        'time': new_time,
        'w_class': new_w_class
    }
    set_booking(updated_booking)

    return f"Booking updated to {new_date} at {new_time} for {new_w_class}." 


def handle_booking_cancellation():
    current_booking = get_booking()
    
    # Check if the user has an existing booking
    if current_booking is None:
        return "You don't have an existing booking to cancel."

    # Show current booking and confirm cancellation
    print("Bot:", output_booking())
    if not confirm_booking_action("Are you sure you want to cancel this booking?"):
        return f"Okay {get_user_name()}, your booking will remain active!" 
    
    # Read the CSV file to remove the booking
    rows_to_keep = []
    booking_found = False
    
    with open('bookings.csv', mode='r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        
        for row in reader:
            # Check if this is the booking we want to cancel
            if (row[0].strip() == current_booking['user'].strip() and 
                row[1].strip() == current_booking['date'].strip() and 
                row[2].strip() == current_booking['time'].strip() and 
                row[3].strip().lower() == current_booking['w_class'].strip().lower()):
                booking_found = True
                continue  # Skip this row (effectively removing it)
            rows_to_keep.append(row)

    if not booking_found:
        return "Could not find your booking in the system."

    # Write the remaining bookings back to the CSV file
    with open('bookings.csv', mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(rows_to_keep)

    # Clear the in-memory booking
    set_booking(None)

    return f"Your booking for {current_booking['w_class']} on {current_booking['date']} at {current_booking['time']} has been cancelled."  