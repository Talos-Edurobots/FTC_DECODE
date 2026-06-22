# Code documentation

## TeleOp
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainTeleOp.java

Αυτό είναι το TeleOp του ρομπότ. Όπως όλα τα αγωνιστικά προγράμματα είναι δομημένο με τρόπο ώστε να 
είναι εύκολο να χωριστεί σε RED - BLUE alliance, γιατί οι κλάσεις MainRed.java, MainBlue.java κληρονομούν 
από την MainTeleOp.java. Μέσα στην κλάση εκτελούνται τα subsystems και συνδυάζονται μεταξύ τους με Inputs
από τους DRIVERS. 

## Auto
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/auto

Αυτό είναι το Auto package. Στόχος του η αποθήκευση των autonomous προγραμμάτων του ρομπότ. Όπως και 
στο TeleOp, τα προγράμματα είναι χωρισμένα σε RED - BLUE alliance, και οι αλλαγές στον σκελετό κάθε προγράμματος
επηρεάζουν και τα δύο alliance ταυτόχρονα. Μέσα σε αυτό το package υπάρχει το old subpackage, το οποίο 
περιέχει τα παλιά autonomous προγράμματα του ρομπότ, τα οποία δεν χρησιμοποιούνται πλέον.

Τα autonoumous επίσης είναι δομημένα με πολλά state machines, ώστε ο κώδικας να είναι πιο ευέλικτος και
κατανοητός. Π.χ. στο AutoV2.java (15μπαλο) έχει ξεχωριστό state machine το shooter, ξεχωριστό το cycle 
με το gate και ξεχωριστό με το κύριο state machine, του οποίου ο μεγαλήτερος ρόλος είναι να καλεί με 
την σωστή σειρά τα υπόλοιπα state machines αλλά και άλλες λειτουργίες του ρομπότ, όπως κάποια subsystems.

Να διαβάσετε τον κώδικα του ρομποτ, και όπου δείτε subsystem θα σκεύτεστε με την λογική ποιά λειτουργία 
του ρομπότ αντιπροσωπεύει. Εξάλλου, αυτός είναι ένας από τους λόγους που έφτιαξα τα subsystems.

Είναι λογικό να σας φαίνεται περίεργος ο κώδικας, γιατί δεν έχει την δομή που συνηθίζεται στο FTC
(όχι extends LinearOpMode, όχι @TeleOp/@Autonomous annotations στους σκελετουύς, κλπ), όμως να σκευτείτε
πως αυτά υπάρχουν στα implementation των σκελετών (δείτε TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/main/MainBlue.java)
και ότι αν δεν είχα όλες αυτές τις κλάσεις, ο κώδικας θα ήταν σαν τον περσυνό για ολλανδία (700+ γραμμές σε ένα αρχείο).
Όμως πίσω ακόμα και από τα subsystems υπάρχουν πολλές ακόμα κλάσεις για ocntrol theory, οπότε το αρχείο 
θα έβγαινε περίπου 15k+ γραμμές, και θα ήταν αδύνατο να το διαβάσει κάποιος και θα ήταν πολύ δυσκολή 
η διαχείρησή του σε όλα τα auto/teleop. Αυτό που προτείνω είναι να καταλάβετε το mental model αυτού του 
repo και αυτό είναι ένα από τα βασικότερα για την σωστή κατανόηση.