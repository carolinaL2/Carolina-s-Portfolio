class FitnessClasses:
    def __init__(self):
        self.classes = {
            'yoga': {
                'name': 'Yoga Flow',
                'price': 20,
                'capacity': 15,
                'description': 'Harmonize body and mind with flowing movements and deep stretches',
                'duration': '60 minutes',
                'intensity': 'Low to Medium',
                'benefits': ['Flexibility', 'Balance', 'Stress reduction', 'Core strength'],
                'suitable_for': 'All levels'
            },
            'pilates': {
                'name': 'Core Pilates',
                'price': 25,
                'capacity': 15,
                'description': 'Focus on core strength, posture, and controlled movements',
                'duration': '50 minutes',
                'intensity': 'Low to Medium',
                'benefits': ['Core strength', 'Posture', 'Flexibility', 'Balance'],
                'suitable_for': 'All levels'
            },
            'spin': {
                'name': 'Spin Cycle',
                'price': 25,
                'capacity': 12,
                'description': 'High-energy indoor cycling class with music and varying intensities',
                'duration': '45 minutes',
                'intensity': 'Medium to High',
                'benefits': ['Cardiovascular fitness', 'Leg strength', 'Low impact', 'Endurance'],
                'suitable_for': 'All levels'
            },
            'hiit': {
                'name': 'High Intensity Interval Training',
                'price': 30,
                'capacity': 20,
                'description': 'Dynamic workout alternating between intense exercises and short rest periods',
                'duration': '45 minutes',
                'intensity': 'High',
                'benefits': ['Fat burning', 'Cardiovascular fitness', 'Strength', 'Endurance'],
                'suitable_for': 'Intermediate to Advanced'
            },
            'zumba': {
                'name': 'Zumba Dance Fitness',
                'price': 25,
                'capacity': 25,
                'description': 'Latin-inspired dance workout that combines fun choreography with cardio',
                'duration': '55 minutes',
                'intensity': 'Medium',
                'benefits': ['Cardiovascular fitness', 'Coordination', 'Full body workout', 'Mood boost'],
                'suitable_for': 'All levels'
            }
        }


    """ ----------------- Getters and Setters ----------------- """ 

    def get_class(self, class_type):
        """Retrieve information for a specific class type."""
        return self.classes.get(class_type)

    def get_all_classes(self):
        """Return all class information."""
        return self.classes

    def get_class_names(self):
        """Return a list of all available class types."""
        return list(self.classes.keys())

    def get_class_capacity(self, class_type):
        """Return the capacity for a specific class type."""
        class_info = self.get_class(class_type)
        return class_info['capacity'] if class_info else None

    def get_class_price(self, class_type):
        """Return the price for a specific class type."""
        class_info = self.get_class(class_type)
        return class_info['price'] if class_info else None

    def class_exists(self, class_type):
        """Check if a class type exists."""
        return class_type in self.classes 