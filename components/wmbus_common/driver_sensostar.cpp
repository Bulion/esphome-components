/*
 Copyright (C) 2020-2022 Patrick Schwarz (gpl-3.0-or-later)
 Copyright (C)      2022 Fredrik Öhrström (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include"meters_common_implementation.h"

namespace
{
    struct Driver : public virtual MeterCommonImplementation {
        Driver(MeterInfo &mi, DriverInfo &di);
    };

    static bool ok = registerDriver([](DriverInfo&di)
    {
        di.setName("sensostar");
        di.setDefaultFields(
            "name,id,status,total_kwh,total_water_m3,target_date,target_kwh,timestamp");
        di.setMeterType(MeterType::HeatMeter);
        di.addLinkMode(LinkMode::C1);
        di.addLinkMode(LinkMode::T1);
        di.addDetection(MANUFACTURER_EFE, 0x04,  0x00);
        di.setConstructor([](MeterInfo& mi, DriverInfo& di){ return std::shared_ptr<Meter>(new Driver(mi, di)); });
    });

    Driver::Driver(MeterInfo &mi, DriverInfo &di) : MeterCommonImplementation(mi, di)
    {
        addOptionalLibraryFields("fabrication_no,meter_datetime,model_version,on_time_h,parameter_set");

        addNumericFieldWithExtractor(
            "total",
            "The total energy consumption recorded by this meter.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Energy,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyEnergyVIF)
            );

        addNumericFieldWithExtractor(
            "total_tariff{tariff_counter}",
            "The total heat energy consumption recorded by this meter on tariff 2-3.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Energy,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyEnergyVIF)
            .set(TariffNr(2), TariffNr(3))
            );

        addNumericFieldWithExtractor(
            "total_subunit{subunit_counter}",
            "The total volume consumption recorded by this meter on subunit 1-3.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Volume,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            .set(SubUnitNr(1), SubUnitNr(3))
            );

        addNumericFieldWithExtractor(
            "total_water_tariff2",
            "The total volume of heating media recorded by this meter on tariff2.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Volume,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            .set(TariffNr(2))
            );

        addNumericFieldWithExtractor(
            "target_water",
            "The total volume of heating media as recorded at the end of the billing period.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Volume,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            .set(StorageNr(1))
            );

        addNumericFieldWithExtractor(
            "target_tariff{tariff_counter}",
            "The energy consumption at the last billing period date on tariff 2-3.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Energy,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyEnergyVIF)
            .set(StorageNr(1))
            .set(TariffNr(2), TariffNr(3))
            );

        addNumericFieldWithExtractor(
            "target_subunit{subunit_counter}",
            "The volume consumption at the last billing period date on subunit 1-3.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Volume,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            .set(StorageNr(1))
            .set(SubUnitNr(1), SubUnitNr(3))
            );

        addNumericFieldWithExtractor(
            "power",
            "The active power consumption.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Power,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyPowerVIF)
            );

        addNumericFieldWithExtractor(
            "power_max",
            "The maximum power consumption over ?period?.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Power,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Maximum)
            .set(VIFRange::AnyPowerVIF)
            );

        addNumericFieldWithExtractor(
            "flow_water",
            "The flow of water.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Flow,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::VolumeFlow)
            );

        addNumericFieldWithExtractor(
            "flow_water_max",
            "The maximum forward flow of water over a ?period?.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Flow,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Maximum)
            .set(VIFRange::VolumeFlow)
            );

        addNumericFieldWithExtractor(
            "forward",
            "The forward temperature of the water.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Temperature,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::FlowTemperature)
            );

        addNumericFieldWithExtractor(
            "return",
            "The return temperature of the water.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Temperature,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::ReturnTemperature)
            );

        addNumericFieldWithExtractor(
            "difference",
            "The temperature difference forward-return for the water.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Temperature,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::TemperatureDifference)
            );

        addNumericFieldWithExtractor(
            "total_water",
            "The total amount of water that has passed through this meter.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Volume,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Volume)
            );

        addStringFieldWithExtractorAndLookup(
            "status",
            "Meter status from tpl field.",
            DEFAULT_PRINT_PROPERTIES | PrintProperty::INCLUDE_TPL_STATUS | PrintProperty::STATUS,
            FieldMatcher::build()
            .set(VIFRange::ErrorFlags),
            {
                {
                    {
                        "ERROR_FLAGS",
                        Translate::MapType::BitToString,
                        AlwaysTrigger, MaskBits(0xff),
                        "OK",
                        {
                            // based on information published here: https://www.engelmann.de/wp-content/uploads/2022/10/1080621004_2022-10-12_BA_S3_ES_Comm_en.pdf
                            { 0x01 , "ERROR_TEMP_SENSOR_1_CABLE_BREAK" },
                            { 0x02 , "ERROR_TEMP_SENSOR_1_SHORT_CIRCUIT" },
                            { 0x04 , "ERROR_TEMP_SENSOR_2_CABLE_BREAK" },
                            { 0x08 , "ERROR_TEMP_SENSOR_2_SHORT_CIRCUIT" },
                            { 0x10 , "ERROR_FLOW_MEASUREMENT_SYSTEM_ERROR" },
                            { 0x20 , "ERROR_ELECTRONICS_DEFECT" },
                            { 0x40 , "OK_INSTRUMENT_RESET" },
                            { 0x80 , "OK_BATTERY_LOW" }
                        }
                    },
                },
            });

        addNumericFieldWithExtractor(
            "target",
            "The energy consumption at the last billing period date.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Energy,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyEnergyVIF)
            .set(StorageNr(1))
            );

        addStringFieldWithExtractor(
            "target_date",
            "The target date for the target energy values.",
            DEFAULT_PRINT_PROPERTIES,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::Date)
            .set(StorageNr(1))
            );

        addNumericFieldWithExtractor(
            "target_{storage_counter}",
            "Target energy values indexed by storage number.",
            DEFAULT_PRINT_PROPERTIES,
            Quantity::Energy,
            VifScaling::Auto, DifSignedness::Signed,
            FieldMatcher::build()
            .set(MeasurementType::Instantaneous)
            .set(VIFRange::AnyEnergyVIF)
            .set(StorageNr(2), StorageNr(32))
            );
    }
}

// Test: Heat sensostar 20480057 NOKEY
// Comment:
// telegram=|68B3B36808007257004820c51400046c100000047839803801040600000000041300000000042B00000000142B00000000043B00000000143B00000000025B1400025f15000261daff02235c00046d2c2ddc24440600000000441300000000426c000001fd171003fd0c05000084200600000000c420060000000084300600000000c430060000000084401300000000c44013000000008480401300000000c48040130000000084c0401300000000c4c0401300000000a216|
// {"media":"heat","meter":"sensostar","name":"Heat","id":"20480057","meter_datetime":"2022-04-28 13:44","total_kwh":0,"power_kw":0,"power_max_kw":0,"flow_water_m3h":0,"flow_water_max_m3h":0,"forward_c":20,"return_c":21,"difference_c":-0.38,"total_water_m3":0,"status":"ERROR_FLOW_MEASUREMENT_SYSTEM_ERROR","target_date":"2000-00-00","target_kwh":0,"timestamp":"1111-11-11T11:11:11Z"}
// |Heat;20480057;ERROR_FLOW_MEASUREMENT_SYSTEM_ERROR;0;0;2000-00-00;0;1111-11-11 11:11.11

// Test: WMZ sensostar 02752560 NOKEY
// Comment: from "Sensostar U"
//telegram=a444c5146025750200047ac20000202f2f046d2e26c62a040643160000041310f0050001fd1700426cbf2c4406570e00008401061f160000840206f6150000840306f5150000840406f3150000840506ea150000840606bf1500008407065214000084080692120000840906c5100000840a06570e0000840b06ca0b0000840c06da090000840d06ca080000840e06c8080000840f06c608000003fd0c05010002fd0b2111
//{"media":"heat","meter":"sensostar","name":"WMZ","id":"02752560","meter_datetime":"2022-10-06 06:46","total_kwh":5699,"total_water_m3":389.136,"status":"OK","target_date":"2021-12-31","target_kwh":3671,"target_2_kwh":5622,"target_3_kwh":5621,"target_4_kwh":5619,"target_5_kwh":5610,"target_6_kwh":5567,"target_7_kwh":5202,"target_8_kwh":4754,"target_9_kwh":4293,"target_10_kwh":3671,"target_11_kwh":3018,"target_12_kwh":2522,"target_13_kwh":2250,"target_14_kwh":2248,"target_15_kwh":2246,"timestamp":"1111-11-11 11:11.11"}
//WMZ;02752560;OK;5699;389.136000;2021-12-31;3671;1111-11-11 11:11.11
void wmbus_anchor_driver_sensostar(void) {}
