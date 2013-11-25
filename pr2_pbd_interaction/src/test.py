#from Action import Action

#a1 = Action()
#a1.type = Action.POSE
#a1.pose = { "position" : { "x" : 10, "y" : 2, "z" : 5 },
            #"orientation" : { "x" : 1, "y" : 1, "z" : 5, "w" : 9 } }

#a2 = Action(0)
            
#act = Action()
#act.type = Action.ACTION_QUEUE
#act.actions = [a1, a2]
#act.save()

#act = Action(1)
#print(map(lambda a: a.pose if a.type == Action.POSE else "Action " + str(a.type), act.actions))

#a1 = Action()
#a1.type = Action.POSE
#a1.pose = { "position" : { "x" : 10, "y" : 2, "z" : 5 },
            #"orientation" : { "x" : 1, "y" : 1, "z" : 5, "w" : 9 } }

#a2 = Action()
#a2.type = Action.POSE
#a2.pose = { "position" : { "x" : 3, "y" : 22, "z" : 51 },
            #"orientation" : { "x" : 12, "y" : 13, "z" : 25, "w" : 93 } }
            
#a3 = Action()
#a3.type = Action.POSE
#a3.pose = { "position" : { "x" : 10, "y" : 2, "z" : 5 },
            #"orientation" : { "x" : 1, "y" : 1, "z" : 5, "w" : 9 } }

#act = Action()
#act.type = Action.ACTION_QUEUE
#act.actions = [a1, a2, a3]
#act.save()