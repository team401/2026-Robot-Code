package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.RoutineCall;
import frc.robot.auto.general.Sequence;

@JsonType(
    property = "type",
    subtypes = {
        @JsonSubtype(clazz = RoutineCall.class, name = "RoutineCall"),
        @JsonSubtype(clazz = Deadline.class, name = "Deadline"),
        @JsonSubtype(clazz = Sequence.class, name = "Sequence"),
        @JsonSubtype(clazz = Parallel.class, name = "Parallel"),
        @JsonSubtype(clazz = Race.class, name = "Race")
    }
)
public abstract class AutoAction {
    
    public String type;
    

    public abstract Command toCommand();

}
