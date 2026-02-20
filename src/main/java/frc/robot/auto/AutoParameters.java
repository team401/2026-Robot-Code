package frc.robot.auto;

import java.util.HashMap;
import java.util.Objects;

public class AutoParameters {
    
    public HashMap<String, AutoParameter<?>> parameters = new HashMap<>();

    public void registerParameter(AutoParameter<?> parameter) {
        Objects.requireNonNull(parameter, "Parameter cannot be null");
        var name = parameter.getName();
        Objects.requireNonNull(name, "The name of the parameter cannot be null");
        if (parameters.containsKey(name)) {
            var existingParameter = parameters.get(name);
            if (existingParameter == null) {
                // This case should never happen, but we check for it just in case
                // And let if does happen we just let it be overridden, since the existing parameter is null
            }else if (existingParameter.getClass() != parameter.getClass()) {
                throw new IllegalArgumentException("Parameter with name " + name + " already exists with a different type");
            } else if (existingParameter.equals(parameter)) {
                return;
            }else{
                throw new IllegalArgumentException("Parameter with name " + name + " already exists");
            }
        }
        parameters.put(name, parameter);
    }

    public void registerIfAbsent(String name, AutoParameter<?> parameter) {
        if (!parameters.containsKey(name)) {
            parameters.put(name, parameter);
        }
    }

    public AutoParameter<?> getParameter(String name) {
        return parameters.get(name);
    }

    public void publishToDashboard() {
        for (var parameter : parameters.values()) {
            parameter.publishToDashboard();
        }
    }

    public static abstract class AutoParameter<T> {
        private final String name;
        public T defaultValue;

        public AutoParameter(String name, T defaultValue) {
            this.name = name;
            this.defaultValue = defaultValue;
        }

        public String getName() {
            return name;
        }

        public abstract T getValue();

        public abstract void setValue(T value);

        public abstract void publishToDashboard();

        public boolean equals(AutoParameter<?> other) {
            if (other == null) return false;
            if (this.getClass() != other.getClass()) return false;
            if (!this.name.equals(other.name)) return false;
            return Objects.equals(this.defaultValue, other.defaultValue);
        }
    }

    public static class ChoiceParameter<T> extends AutoParameter<T> {
        public HashMap<String, T> choices;

        public ChoiceParameter(String name, T defaultValue, HashMap<String, T> choices) {
            super(name, defaultValue);
            this.choices = choices;
        }

        @Override
        public T getValue() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getValue'");
        }

        @Override
        public void setValue(T value) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setValue'");
        }

        @Override
        public void publishToDashboard() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'publishToDashboard'");
        }

        public boolean equals(AutoParameter<?> other) {
            if (!super.equals(other)) return false;
            // we know that other is a ChoiceParameter because of the super.equals check, so we can safely cast it
            var otherChoice = (ChoiceParameter<?>) other;
            return Objects.equals(this.choices, otherChoice.choices);
        }

    }

    public static class NumberParameter extends AutoParameter<Double> {
        public double min;
        public double max;

        public NumberParameter(String name, Double defaultValue, double min, double max) {
            super(name, defaultValue);
            this.min = min;
            this.max = max;
        }

        @Override
        public Double getValue() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getValue'");
        }

        @Override
        public void setValue(Double value) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setValue'");
        }

        @Override
        public void publishToDashboard() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'publishToDashboard'");
        }

        public boolean equals(AutoParameter<?> other) {
            if (!super.equals(other)) return false;
            // we know that other is a NumberParameter because of the super.equals check, so we can safely cast it
            var otherNumber = (NumberParameter) other;
            return this.min == otherNumber.min && this.max == otherNumber.max;
        }
    }

    public static class BooleanParameter extends AutoParameter<Boolean> {
        public BooleanParameter(String name, Boolean defaultValue) {
            super(name, defaultValue);
        }

        @Override
        public Boolean getValue() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getValue'");
        }

        @Override
        public void setValue(Boolean value) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setValue'");
        }

        @Override
        public void publishToDashboard() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'publishToDashboard'");
        }
    }

    public static class StringParameter extends AutoParameter<String> {
        public StringParameter(String name, String defaultValue) {
            super(name, defaultValue);
        }

        @Override
        public String getValue() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getValue'");
        }

        @Override
        public void setValue(String value) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'setValue'");
        }

        @Override
        public void publishToDashboard() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'publishToDashboard'");
        }
    }

}
