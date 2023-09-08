# Procedural Hand Movement
Procedurally animated robot hand project. The project has 2 variation, semi-active and full-active. Semi-active hand is more agile than full-active hand but semi-acitve is not realistic. full-active is very realistic hand therefore agile movement is not much possible in current algorithm.<br>

Demo project uses new and old input system.<br>
Movement: WASD<br>
Aim: Mouse<br>
Focus: Mouse Right Button<br>
Fire: Mouse Left Button<br>
C: Lock/Release cursor<br>
Zoom in/out: Mouse Scroll

## Semi-active Hand
![Semi-active hand movement showcase](README_ASSETS/SemiHand.gif)<br>
This variation is suitable for arcade type games. Less realistic but agile movement.

## Full-active Hand
![Full-active hand movement showcase](README_ASSETS/FullHand.gif)<br>
This variation is suitable for realistic games. It is not realistic enough to use it in simulation games.

## Changing the hand model

If you want to change the hand model with yours. I listed important steps below.

- Make sure your hand model doesn't use armature. This will cause problem on firing the finger segment. Instead of disabling GameObject, bone will disabled and finger segment will be visible.
- Make sure you have two same hand object, one is active other one is kinematic.
- Only active object must have physics components (such as Collider, RigidBody, Joint etc.)
- Only kinematic object must have IK components (such as Rig, Animator etc.)

Setting Active object:
- Add Rigidbody and player controller components to this object.
- Add necessary collider components.
- Add each finger these:
  - RigidBody (Set mass realisticly)
  - Collider (In the project only tip of the finger has capsule, other segments have box)
  - Joint (Recommended Configurable Joint)
  - If you are planning to use full active controller, please add CollisionDetector component. Assign FullTargetSolver.TipCollisionEnter to On Collision Enter event and FullTargetSolver.TipCollisionExit to On Collision Exit event.
- Don't forget to set joint parameters. If you need reference you can look default hand model in the project. Make sure joint anchor is correct.
- If you are planning to use full active controller, add empty object to the last finger segments. It will be used as finger tip so place it on the finger tip. And don't forget to disable finger tip gameobject.

Setting Kinematic object:
- Add Animator and IKHolder components to this object.
- While this object selected you can either select Animation Rigging>Rig Setup on top left corner or manually adding RigBuilder component to this object.
- For each finger create new GameObject in "Rig" GameObject which is setted on RigBuilder component and then add these components:
  - Chain IK Constraint (Set parameters according to your model)
  - Multi-Aim Constraint (Set parameters according to your model, make sure weight is 0)
  - Target IK Solver
  - IK Info
- Adjust each IK GameObject's right axis (red arrow) to be tangent with the attached surface
- Within each IK GameObject create 2 GameObject as target and raycast. No need to adjust target GameObject. Face Raycast GameObject to where should fingers check the surface, blue line will be the raycast (If you can't see blue line make sure you set Ray Length on Target IK Solver component to bigger than 0 and toggled gizmos).
- Now you are ready to set parameters.

## Limitations
- Firing finger segments is not possible with Skinned Mesh Renderer. Therefore hands with bones (armature) is not supported. If you don't intend to fire finger segments, then there won't be any problem using hand model with armature.
- Scripts strictly written to be used in quadrupeds. Bipeds are not supported and may require modification of the IK solver script.
- Make full-active hand to move faster is hard to achieve. Due to the physics move/rotation limitations, it is hard to achieve faster or agile movement like semi-active hand.
- Currently, full-active hand can fall while aiming. I tried to calculate shifted center of mass and move fingers according to that center of mass but failed. I am open for solution to this problem.

## Known Issues
- Full-active IK solver script is way complicated to understand. Feel free to open issues or mail me if you have any questions.
