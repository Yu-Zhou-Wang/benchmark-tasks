"""Fixed scene, dimensions in metres, and the policy instruction."""
TOP=.7475
BOTTLE_XY=(.24,.20)
BOTTLE_HEIGHT=.100
CAP_START_Z=TOP+.083
CAP_RADIUS=.029
CAP_HEIGHT=.027
PAD_XY=(.32,0.)
PAD_SIZE=(.036,.036,.050)
PAD_TOP=TOP+PAD_SIZE[2]
MAX_STEPS=1000
INSTRUCTION=("The blue reagent-bottle cap is already unscrewed and resting loosely on the neck. "
 "Lift it off and park it in the centre of the green pad with its hollow interior facing upward. "
 "Touch only the cap's exterior: keep its inner surface and opening rim away from both grippers, "
 "the bench and the pad throughout. Leave the cap released and stable for two seconds. "
 "The empty bottle is secured in a bench holder. No unscrewing is needed.")
