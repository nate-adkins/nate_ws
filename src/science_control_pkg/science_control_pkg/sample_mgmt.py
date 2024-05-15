from dynamixel_lib import Dynamixel, XL430

class SampleMgmt():

    def __init__(self, dynamixel_id, u2d2):
        self.dynamixel = Dynamixel(XL430, dynamixel_id, u2d2)
        self.encoder_counts = 4096
        self.cuvette_indices = 12

    def cuvette_index_to_motor_position(self, index: int) -> int:
        """_summary_

        Args:
            index (int): index of the cuvette desired

        Returns:
            int: _description_
        """        
        steps = [(int(self.encoder_counts / self.cuvette_indices) * (n)) for n in range(0,self.cuvette_indices,1)]
        return steps[index]


    
    def align_index_and_function(self):


    
