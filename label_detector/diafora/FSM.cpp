enum states {      // Define the states in the state machine.
  NO_PIZZA,        // Exit state machine.
  COUNT_PEOPLE,    // Ask user for # of people.
  COUNT_SLICES,    // Ask user for # slices.
  SERVE_PIZZA,     // Validate and serve.
  EAT_PIZZA        // Task is complete.
} STATE;

STATE state = COUNT_PEOPLE;
int nPeople, nSlices, nSlicesPerPerson;

// Serve slices of pizza to people, so that each person gets
/// the same number of slices.   
while (state != NO_PIZZA)  {
   switch (state)  {
   case COUNT_PEOPLE:  
       if (promptForPeople(&nPeople))  // If input is valid..
           state = COUNT_SLICES;       // .. go to next state..
       break;                          // .. else remain in this state.
   case COUNT_SLICES:  
       if (promptForSlices(&nSlices))
          state = SERVE_PIZZA;
        break;
   case SERVE_PIZZA:
       if (nSlices % nPeople != 0)    // Can't divide the pizza evenly.
       {                             
           getMorePizzaOrFriends();   // Do something about it.
           state = COUNT_PEOPLE;      // Start over.
       }
       else
       {
           nSlicesPerPerson = nSlices/nPeople;
           state = EAT_PIZZA;
       }
       break;
   case EAT_PIZZA:
       // etc...
       state = NO_PIZZA;  // Exit the state machine.
       break;
   } // switch
} // while