* switch the lost iteration count to use max_weight in some way
* make the distance sensor pdf use another exponential centered at the expected distance - should make predictions better
* new pdf makes difference between particles not as big, seems to be an issue for the prediction
    * could try different methods of prediction
    * could change pdf to reduce the random noise uniform distribution, however this makes the integral smaller
        * the second exponential might help with this
