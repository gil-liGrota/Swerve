package frc.lib.tunables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import frc.lib.valueholders.BooleanHolder;
import frc.lib.valueholders.DoubleHolder;
import frc.lib.valueholders.ValueHolder;

public class TunablesTable implements Tunable {
    private final SendableType sendableType;
    private final List<PropertyNode> propertyNodes = new ArrayList<>();
    private final Map<String, Tunable> tunablesNode = new HashMap<>();

    private interface PropertyNode {
        void addToBuilder(TunableBuilder tunableBuilder);
    }

    public TunablesTable(SendableType sendableType) {
        this.sendableType = sendableType;
    }

    public void addChild(String name, Tunable tunable) {
        tunablesNode.put(name, tunable);
    }

    public void addChild(String name, Sendable sendable) {
        tunablesNode.put(name, sendable::initSendable);
    }

    public void addBoolean(String key, BooleanSupplier getter, BooleanConsumer setter) {
        propertyNodes.add((builder) -> builder.addBooleanProperty(key, getter, setter));
    }

    public BooleanHolder addBoolean(String key, boolean defaultValue) {
        BooleanHolder refValue = new BooleanHolder(defaultValue);
        addBoolean(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addNumber(String key, DoubleSupplier getter, DoubleConsumer setter) {
        propertyNodes.add((builder) -> builder.addDoubleProperty(key, getter, setter));
    }

    public DoubleHolder addNumber(String key, Double defaultValue) {
        DoubleHolder refValue = new DoubleHolder(defaultValue);
        addNumber(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addString(String key, Supplier<String> getter, Consumer<String> setter) {
        propertyNodes.add((builder) -> builder.addStringProperty(key, getter, setter));
    }

    public ValueHolder<String> addString(String key, String defaultValue) {
        ValueHolder<String> refValue = new ValueHolder<String>(defaultValue);
        addString(key, refValue::get, refValue::set);
        return refValue;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        if (sendableType != SendableType.NONE) {
            builder.setSmartDashboardType(sendableType.getStringType());
        }

        for (PropertyNode propertyNode : propertyNodes) {
            propertyNode.addToBuilder(builder);
        }

        tunablesNode.forEach((name, tunable) -> {
            builder.addChild(name, tunable);
        });
    }
}
