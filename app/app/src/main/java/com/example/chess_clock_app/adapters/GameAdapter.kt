package com.example.chess_clock_app.adapters

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageButton
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.models.Game

class GameAdapter(
    private val games: List<Game>,
    private val onGameClicked: (Game) -> Unit,
    private val onGameSettingsClicked: (Game) -> Unit
) : RecyclerView.Adapter<GameAdapter.ViewHolder>()  {

    //How a single member of the list looks like
    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {

        val gameName: TextView =
            view.findViewById(R.id.gameName)

       // val settingsButton: ImageButton =
       //     view.findViewById(R.id.gameSettingsButton)
    }
    override fun onCreateViewHolder(
        parent: ViewGroup,
        viewType: Int
    ): ViewHolder {
        val view = LayoutInflater
            .from(parent.context)
            .inflate(R.layout.item_game, parent, false)

        return ViewHolder(view)
    }

    override fun onBindViewHolder(
        holder: ViewHolder,
        position: Int
    ) {
        val game = games[position]

        holder.gameName.text = game.name

        holder.itemView.setOnClickListener {
            onGameClicked(game)
        }

        //holder.settingsButton.setOnClickListener {
        //    onGameSettingsClicked(game)
        //}
    }
    override fun getItemCount(): Int {
        return games.size
    }
}