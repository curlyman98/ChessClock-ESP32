package com.example.chess_clock_app.activities

import android.os.Bundle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.adapters.GameAdapter
import com.example.chess_clock_app.models.Game
import android.widget.Toast
import android.view.View
import android.widget.Button
import android.widget.ImageButton
import android.widget.ProgressBar
import android.content.Intent
import androidx.activity.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.example.chess_clock_app.utils.setEnabledLook
import com.example.chess_clock_app.utils.showMessage
import com.example.chess_clock_app.viewmodels.GameListViewModel
import com.example.chess_clock_app.viewmodels.GameSortOrder
import kotlinx.coroutines.launch
//import android.widget.TextView

class GameActivity : BaseActivity() {

    // Same instance survives rotation; created through the Factory the first time
    private val viewModel: GameListViewModel by viewModels { GameListViewModel.Factory }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Insert pairing-specific UI into activity_base.xml.
        setActivityContent(R.layout.content_games)

        val recyclerView =
            findViewById<RecyclerView>(R.id.gamesRecyclerView)

        recyclerView.layoutManager =
            LinearLayoutManager(this)

        // Re-runs every time the games table changes; only while the screen is visible
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.games.collect { games ->
                    showGames(recyclerView, games)
                }
            }
        }

        // Shows the current sort: calendar = by date, A-Z = by name
        val sortButton =
            findViewById<ImageButton>(R.id.sortButton)

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.sortOrder.collect { order ->
                    sortButton.setImageResource(
                        if (order == GameSortOrder.NAME) R.drawable.ic_sort_by_alpha else R.drawable.ic_sort_by_date
                    )
                }
            }
        }

        sortButton.setOnClickListener {
            viewModel.toggleSortOrder()
            val text = if (viewModel.sortOrder.value == GameSortOrder.NAME) R.string.sorted_by_name else R.string.sorted_by_date
            Toast.makeText(this, text, Toast.LENGTH_SHORT).show()
        }

        val syncButton =
            findViewById<Button>(R.id.syncButton)

        val scanningIndicator =
            findViewById<ProgressBar>(R.id.syncingIndicator)

        // Sync is greyed out while no chess clock is connected
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.isConnected.collect { connected ->
                    syncButton.setEnabledLook(connected)
                }
            }
        }

        // Opens the edit screen without a game key, which means "new game"
        val addGameButton =
            findViewById<Button>(R.id.addGameButton)

        addGameButton.setOnClickListener {
            startActivity(Intent(this, GameEditActivity::class.java))
        }

        syncButton.setOnClickListener {
            if (viewModel.isSyncing.value) viewModel.cancelSync() else viewModel.sync()
        }

        // Spinner and button text follow the sync, also when it ends by itself
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.isSyncing.collect { syncing ->
                    scanningIndicator.visibility = if (syncing) View.VISIBLE else View.GONE
                    syncButton.text = getString(if (syncing) R.string.cancel_sync else R.string.sync)
                }
            }
        }

        // "Games imported: 2" etc.
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.messages.collect { showMessage(it) }
            }
        }

    }

    private fun showGames(recyclerView: RecyclerView, games: List<Game>) {
        recyclerView.adapter = GameAdapter(
            games = games,

            onGameClicked = { games ->
                val intent = Intent(this, GameEditActivity::class.java)

                intent.putExtra(GameEditActivity.EXTRA_GAME, games.key)

                startActivity(intent)

            },
            onGameSettingsClicked = { games ->
                Toast.makeText(
                    this,
                    "Settings for game ${games.name}",
                    Toast.LENGTH_SHORT
                ).show()

            }

        )
    }
}
