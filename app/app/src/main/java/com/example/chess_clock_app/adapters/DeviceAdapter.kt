package com.example.chess_clock_app.adapters

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageButton
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.models.Device

class DeviceAdapter(
    private val devices: List<Device>,
    private val onDeviceClicked: (Device) -> Unit,
    // null = no edit button, e.g. for nearby devices that are not paired yet
    private val onDeviceSettingsClicked: ((Device) -> Unit)? = null,
    // Drawn in a different colour; null = none connected
    private val connectedMac: String? = null,
): RecyclerView.Adapter<DeviceAdapter.ViewHolder>() {

    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {

        val deviceName: TextView =
            view.findViewById(R.id.deviceName)

        val settingsButton: ImageButton =
            view.findViewById(R.id.deviceSettingsButton)
    }

    override fun onCreateViewHolder(
        parent: ViewGroup,
        viewType: Int
    ): ViewHolder {
        val view = LayoutInflater
            .from(parent.context)
            .inflate(R.layout.item_device, parent, false)

        return ViewHolder(view)
    }

    override fun onBindViewHolder(
        holder: ViewHolder,
        position: Int
    ) {
        val device = devices[position]

        holder.deviceName.text = device.name

        holder.itemView.setBackgroundResource(
            if (device.mac == connectedMac) R.drawable.device_item_connected_background
            else R.drawable.device_item_background
        )

        holder.itemView.setOnClickListener {
            onDeviceClicked(device)
        }

        // remove the edit button from the pairing devices
        holder.settingsButton.visibility =
            if (onDeviceSettingsClicked != null) View.VISIBLE else View.GONE

        holder.settingsButton.setOnClickListener {
            onDeviceSettingsClicked?.invoke(device)
        }
    }

    override fun getItemCount(): Int {
        return devices.size
    }
}